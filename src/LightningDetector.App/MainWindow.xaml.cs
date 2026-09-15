using System.Collections.ObjectModel;
using System.Globalization;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace LightningDetector.App;

public partial class MainWindow : Window
{
    private static readonly TimeSpan PollDeviceInterval = TimeSpan.FromSeconds(5);

    private SerialPort? _serialPort;
    private readonly StringBuilder _pendingData = new();
    private readonly ObservableCollection<DetectionRow> _detections = [];
    private readonly List<string> _manualCommandHistory = [];
    private readonly object _serialSync = new();
    private readonly SemaphoreSlim _serialCommandGate = new(1);
    private TaskCompletionSource<string?>? _pendingResponse;
    private string? _manualCommandHistoryPrefix;
    private bool _isApplyingHistorySelection;
    private DispatcherTimer? _pollingTimer;
    private bool _isInitializing;
    private bool _isApplyingConfiguration;
    private bool _suppressConfigurationUpdates;
    private int _manualCommandHistoryIndex = -1;
    private int _lastLightningCount = -1;
    private int _lastNoiseCount = -1;
    private int _lastDisturberCount = -1;
    private DateTime? _lastLightningDetectionTime;
    private readonly List<Shape> _chartDots = [];
    private bool _isPollingPaused;
    private bool _isSerialPollingEnabled;
    private TimeSpan _serialPollingInterval = TimeSpan.FromSeconds(1);
    private DispatcherTimer? _serialPollingTimer;
    private int _lastStbValue;
    private bool _isReadingStb;
    private string _currentAfe = "?";
    private string _currentNoiseThreshold = "?";
    private string _currentSpikeReject = "?";

    private string _currentWatchdogThreshold = "?";
    private string _currentDisturberState = "?";
    private string _currentLightningThreshold = "?";
    private string _selfTestResult = "Inconnu";
    private string _identity = string.Empty;
    private string _errorsSummary = string.Empty;

    public MainWindow()
    {
        InitializeComponent();
        ManualCommandTextBox.TextChanged += ManualCommandTextBox_TextChanged;
        ManualCommandTextBox.PreviewKeyDown += ManualCommandTextBox_PreviewKeyDown;
        LightningChartCanvas.SizeChanged += (_, _) => RefreshLightningChart();
        DetectionsDataGrid.ItemsSource = _detections;
        LoadPortNames();
        ResetStatusUi();
        UpdatePauseButtonState();
        SelectComboBoxValue(SerialPollingIntervalComboBox, "100ms");
        SelectComboBoxValue(AfeComboBox, "intérieur");
        SelectComboBoxValue(NoiseThresholdComboBox, "2");
        SelectComboBoxValue(WatchdogThresholdComboBox, "2");
        SelectComboBoxValue(SpikeRejectComboBox, "2");
        DisturberStateCheckBox.IsChecked = true;
        SelectComboBoxValue(LightningThresholdComboBox, "1");
        SelectComboBoxValue(ConfigurationSlotComboBox, "0");
        UpdateBargraphAvailability();
    }

    private void LoadPortNames()
    {
        var ports = SerialPort.GetPortNames();
        PortComboBox.ItemsSource = ports;
        if (ports.Length > 0)
        {
            PortComboBox.SelectedIndex = 0;
        }
    }

    private async void ConnectToggleButton_Click(object sender, RoutedEventArgs e)
    {
        if (_serialPort?.IsOpen == true)
        {
            Disconnect();
            return;
        }

        if (_isInitializing)
        {
            return;
        }

        try
        {
            var portName = PortComboBox.SelectedItem?.ToString();
            if (string.IsNullOrWhiteSpace(portName))
            {
                MessageBox.Show("Sélectionnez un port COM.", "Information", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            var selectedBaudRate = ((ComboBoxItem)BaudRateComboBox.SelectedItem).Content?.ToString();
            var baudRate = int.TryParse(selectedBaudRate, out var parsedBaudRate) ? parsedBaudRate : 115200;

            _serialPort = new SerialPort(portName, baudRate)
            {
                ReadTimeout = 2000,
                WriteTimeout = 2000,
                NewLine = "\n"
            };

            _serialPort.DataReceived += SerialPort_DataReceived;
            _serialPort.Open();

            ResetSessionState();
            RawOutputTextBox.Text = string.Empty;
            AppendRaw($"Connecté sur {portName}");
            // When connected, indicate we're waiting for identification and self-test
            ModuleStatusText.Text = "En attente...";
            StatusIndicator.Fill = Brushes.LightGray;
            UpdatePauseButtonState();
            UpdateConnectionDependentUi();

            await EnableSerialPollingAsync();
            await InitializeDeviceAsync();
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Erreur de connexion : {ex.Message}", "Erreur", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void Disconnect()
    {
        StopPollingTimer();
        StopSerialPollingTimer();

        if (_serialPort is { IsOpen: true })
        {
            _serialPort.DataReceived -= SerialPort_DataReceived;
            _serialPort.Close();
        }

        _serialPort = null;

        lock (_serialSync)
        {
            _pendingResponse?.TrySetResult(null);
            _pendingResponse = null;
        }

        AppendRaw("Déconnecté.");
        _isPollingPaused = false;
        UpdatePauseButtonState();
        UpdateConnectionDependentUi();
        ResetSerialPollingState();
        ResetStatusUi();
    }

    private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        try
        {
            Dispatcher.Invoke(() =>
            {
                if (_isReadingStb)
                {
                    return;
                }

                var data = _serialPort?.ReadExisting();
                if (string.IsNullOrEmpty(data))
                {
                    return;
                }

                _pendingData.Append(data);
                ProcessPendingData();
            });
        }
        catch (Exception ex)
        {
            Dispatcher.Invoke(() => AppendRaw($"Erreur : {ex.Message}"));
        }
    }

    private void ProcessPendingData()
    {
        var buffer = _pendingData.ToString();
        var startIndex = 0;

        while (true)
        {
            var newlineIndex = buffer.IndexOf('\n', startIndex);
            if (newlineIndex < 0)
            {
                break;
            }

            var line = buffer.Substring(startIndex, newlineIndex - startIndex).Replace("\r", string.Empty).Trim();
            if (!string.IsNullOrWhiteSpace(line))
            {
                ResolvePendingResponse(line);
            }

            startIndex = newlineIndex + 1;
        }

        _pendingData.Clear();
        if (startIndex < buffer.Length)
        {
            _pendingData.Append(buffer[startIndex..]);
        }
    }

    private void ResolvePendingResponse(string line)
    {
        TaskCompletionSource<string?>? pending = null;
        lock (_serialSync)
        {
            pending = _pendingResponse;
            _pendingResponse = null;
        }

        pending?.TrySetResult(line);
    }

    private async Task<string?> QueryIdentityWithRetryAsync()
    {
        const int maxAttempts = 8;
        for (var attempt = 1; attempt <= maxAttempts; attempt++)
        {
            var identity = await SendCommandAsync("*IDN?", 500);
            if (!string.IsNullOrWhiteSpace(identity))
            {
                return identity;
            }

            if (attempt < maxAttempts)
            {
                AppendRaw($"! pas de réponse à *IDN? (tentative {attempt}/{maxAttempts}), nouvelle tentative dans 500 ms");
                await Task.Delay(500);
            }
        }

        return null;
    }

    private async Task InitializeDeviceAsync()
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        _isInitializing = true;
        try
        {
            var identity = await QueryIdentityWithRetryAsync();
            if (string.IsNullOrWhiteSpace(identity))
            {
                ModuleStatusText.Text = "Pas de réponse";
                StatusIndicator.Fill = Brushes.Red;
                AppendRaw("! Pas de réponse à *IDN? après 8 tentatives.");
                return;
            }

            if (!IsValidIdentity(identity))
            {
                ModuleStatusText.Text = "Capteur non reconnu";
                StatusIndicator.Fill = Brushes.Red;
                AppendRaw($"! Identité inattendue à *IDN? : {identity}");
                return;
            }

            _identity = identity;
            ModuleNameText.Text = identity;
            ModuleStatusText.Text = identity;

            var selfTest = await SendCommandAsync("*TST?", 1000);
            _selfTestResult = selfTest?.Trim().TrimStart('+') == "0" ? "OK" : (selfTest ?? "Erreur");
            UpdateStatusUi();

            CheckStatusAndErrorsAsync();
            await LoadConfigurationAsync();

            StartPollingTimer();
        }
        catch (Exception ex)
        {
            AppendRaw($"Erreur d'initialisation : {ex.Message}");
        }
        finally
        {
            _isInitializing = false;
        }
    }

    private async Task LoadConfigurationAsync()
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        var afe = await SendCommandAsync(":SENSe:AFE:GAIN?", 1000);
        var noise = await SendCommandAsync(":SENSe:NOISe:THReshold?", 1000);
        var reject = await SendCommandAsync(":SENSe:SPIKe:REJection?", 1000);
        var watchdog = await SendCommandAsync(":SENSe:WATChdog:THReshold?", 1000);
        var state = await SendCommandAsync(":SYSTem:DISTurber:STATe?", 1000);
        var lightning = await SendCommandAsync(":SENSe:LIGHtning:THReshold?", 1000);

        _currentAfe = NormalizeAfeResponse(afe);
        _currentNoiseThreshold = NormalizeValue(noise);
        _currentSpikeReject = NormalizeValue(reject);
        _currentWatchdogThreshold = NormalizeValue(watchdog);
        _currentDisturberState = NormalizeBooleanResponse(state);
        _currentLightningThreshold = NormalizeValue(lightning);

        Dispatcher.Invoke(() =>
        {
            try
            {
                _suppressConfigurationUpdates = true;
                SelectComboBoxValue(AfeComboBox, _currentAfe switch
                {
                    "IND" => "intérieur",
                    "OUT" => "extérieur",
                    _ => "intérieur"
                });
                SelectComboBoxValue(NoiseThresholdComboBox, _currentNoiseThreshold);
                SelectComboBoxValue(SpikeRejectComboBox, _currentSpikeReject);
                SelectComboBoxValue(WatchdogThresholdComboBox, _currentWatchdogThreshold);
                DisturberStateCheckBox.IsChecked = _currentDisturberState != "0";
                SelectComboBoxValue(LightningThresholdComboBox, _currentLightningThreshold);
                UpdateConfigurationUi();
            }
            finally
            {
                _suppressConfigurationUpdates = false;
            }
        });
    }

    private void CheckStatusAndErrorsAsync()
    {
        _errorsSummary = "Aucune erreur";
        UpdateConfigurationUi();
    }

    private const double ChartWidth = 300;
    private const double ChartHeight = 150;
    private const double ChartMaxDistanceKm = 40.0;
    private static readonly TimeSpan ChartWindow = TimeSpan.FromMinutes(60);
    private const int LevelBarMaxValue = 100;
    private const double LightningEnergyMax = 2097151.0; // Valeur max 21-bit renvoyée par l'AS3935 (ENERGIE_MAX du firmware)
    private static readonly TimeSpan StormActiveWindow = TimeSpan.FromMinutes(10);

    private async Task PollDeviceAsync()
    {
        if (_serialPort is null || !_serialPort.IsOpen || _isInitializing)
        {
            return;
        }

        var response = await SendCommandAsync(":CALC:NOISE?;DIST?;LIGH?", 2000);
        if (string.IsNullOrWhiteSpace(response))
        {
            AppendRaw("! réponse vide ou timeout lors du polling de l'appareil");
            return;
        }

        var parts = response.Split(';', StringSplitOptions.TrimEntries);
        if (parts.Length < 3)
        {
            AppendRaw($"! réponse de polling non conforme : {response}");
            return;
        }

        var noiseDiff = int.TryParse(parts[0], out var noiseCount) ? ComputeLevelDiff(ref _lastNoiseCount, noiseCount) : (int?)null;
        var disturberDiff = int.TryParse(parts[1], out var disturberCount) ? ComputeLevelDiff(ref _lastDisturberCount, disturberCount) : (int?)null;

        if (int.TryParse(parts[2], out var lightningCount))
        {
            if (_lastLightningCount < 0)
            {
                _lastLightningCount = lightningCount;
            }
            else if (lightningCount > _lastLightningCount)
            {
                var newEvents = lightningCount - _lastLightningCount;
                _lastLightningCount = lightningCount;
                _lastLightningDetectionTime = DateTime.Now;

                for (var index = 0; index < newEvents; index++)
                {
                    var lightningPayload = await SendCommandAsync(":FETCh:ENERgy?;DISTance?", 2000);
                    AddDetection(lightningPayload);
                }
            }
            else
            {
                _lastLightningCount = lightningCount;
            }
        }

        Dispatcher.Invoke(() =>
        {
            if (noiseDiff is { } noise)
            {
                AnimateFillHeight(NoiseLevelFill, NoiseLevelTrack.Height * noise / LevelBarMaxValue);
            }

            if (disturberDiff is { } disturber)
            {
                AnimateFillHeight(DisturberLevelFill, DisturberLevelTrack.Height * disturber / LevelBarMaxValue);
            }

            LightningLegendText.Text = FormatLightningCountText(_lastLightningCount);
            UpdateStormStatus();
            RefreshLightningChart();
        });
    }

    private static string FormatLightningCountText(int count)
    {
        if (count <= 0)
        {
            return "Aucun éclair";
        }

        return count == 1 ? "1 éclair" : $"{count} éclairs";
    }

    private void UpdateStormStatus()
    {
        var isStormActive = _lastLightningDetectionTime is { } lastDetection && DateTime.Now - lastDetection <= StormActiveWindow;
        StormStatusText.Text = isStormActive ? "Orage en cours" : "Aucun orage détecté";
        StormIcon.Fill = isStormActive ? Brushes.Orange : Brushes.Gray;
    }

    private static void AnimateFillHeight(FrameworkElement fillElement, double toHeight)
    {
        var animation = new DoubleAnimation
        {
            To = Math.Max(toHeight, 0),
            Duration = TimeSpan.FromMilliseconds(500),
            EasingFunction = new QuadraticEase { EasingMode = EasingMode.EaseInOut }
        };
        fillElement.BeginAnimation(FrameworkElement.HeightProperty, animation);
    }

    /// <summary>
    /// Compare à la dernière mesure et met à jour la référence. Retourne null pour la toute
    /// première mesure (pas de référence valide pour calculer une différence).
    /// </summary>
    private static int? ComputeLevelDiff(ref int lastValue, int currentValue)
    {
        if (lastValue < 0)
        {
            lastValue = currentValue;
            return null;
        }

        var diff = currentValue - lastValue;
        lastValue = currentValue;
        return Math.Clamp(diff, 0, LevelBarMaxValue);
    }

    private void RefreshLightningChart()
    {
        foreach (var dot in _chartDots)
        {
            LightningChartCanvas.Children.Remove(dot);
        }

        _chartDots.Clear();

        var width = LightningChartCanvas.ActualWidth > 0 ? LightningChartCanvas.ActualWidth : ChartWidth;
        var now = DateTime.Now;
        foreach (var detection in _detections)
        {
            var elapsed = now - detection.Timestamp;
            if (elapsed < TimeSpan.Zero || elapsed > ChartWindow)
            {
                continue;
            }

            if (detection.DistanceKm is not { } distanceKm || double.IsNaN(distanceKm))
            {
                continue;
            }

            var isOutOfRange = double.IsPositiveInfinity(distanceKm) || distanceKm > ChartMaxDistanceKm;
            var clampedDistance = Math.Clamp(isOutOfRange ? ChartMaxDistanceKm : distanceKm, 0, ChartMaxDistanceKm);
            var x = width * (1 - elapsed.TotalMinutes / ChartWindow.TotalMinutes);
            var y = ChartHeight * (1 - clampedDistance / ChartMaxDistanceKm);
            var brush = GetEnergyBrush(detection.Energy);

            Shape dot = isOutOfRange ? CreateOutOfRangeMarker(x, y, brush) : CreateDetectionDot(x, y, brush);
            LightningChartCanvas.Children.Add(dot);
            _chartDots.Add(dot);
        }
    }

    private static Ellipse CreateDetectionDot(double x, double y, Brush brush)
    {
        var ellipse = new Ellipse
        {
            Width = 6,
            Height = 6,
            Fill = brush
        };
        Canvas.SetLeft(ellipse, x - 3);
        Canvas.SetTop(ellipse, y - 3);
        return ellipse;
    }

    /// <summary>
    /// Marqueur pour un éclair à distance inconnue/infinie ou au-delà de la portée du graphique (40 km) :
    /// un triangle pointant vers le haut, positionné comme s'il était à 40 km.
    /// </summary>
    private static Polygon CreateOutOfRangeMarker(double x, double y, Brush brush)
    {
        const double halfWidth = 4;
        const double height = 7;
        var triangle = new Polygon
        {
            Points = [new Point(halfWidth, 0), new Point(0, height), new Point(halfWidth * 2, height)],
            Fill = brush
        };
        Canvas.SetLeft(triangle, x - halfWidth);
        Canvas.SetTop(triangle, y - (height / 2));
        return triangle;
    }

    private static Brush GetEnergyBrush(double? energy)
    {
        if (energy is not { } value || double.IsNaN(value))
        {
            return Brushes.Gray;
        }

        var normalized = Math.Clamp(Math.Sqrt(Math.Max(value, 0) / LightningEnergyMax), 0, 1);
        return normalized < 0.5
            ? new SolidColorBrush(InterpolateColor(Colors.DeepSkyBlue, Colors.Gold, normalized / 0.5))
            : new SolidColorBrush(InterpolateColor(Colors.Gold, Colors.Red, (normalized - 0.5) / 0.5));
    }

    private static Color InterpolateColor(Color from, Color to, double t)
    {
        t = Math.Clamp(t, 0, 1);
        return Color.FromRgb(
            (byte)(from.R + ((to.R - from.R) * t)),
            (byte)(from.G + ((to.G - from.G) * t)),
            (byte)(from.B + ((to.B - from.B) * t)));
    }

    private void AddDetection(string? payload)
    {
        if (string.IsNullOrWhiteSpace(payload))
        {
            return;
        }

        var parts = payload.Split(';', StringSplitOptions.TrimEntries);
        var energy = ParseNullableDouble(parts.Length > 0 ? parts[0] : null);
        var distanceKm = ParseDistanceKm(parts.Length > 1 ? parts[1] : null);

        Dispatcher.Invoke(() =>
        {
            _detections.Add(new DetectionRow
            {
                Timestamp = DateTime.Now,
                DistanceKm = distanceKm,
                Energy = energy
            });
        });
    }

    // Sentinelles utilisées par le capteur pour les valeurs de :FETCh:ENERgy? et :DISTance? :
    // 9.91E+37 signifie "NaN" (valeur inconnue) et 9.90E+37 signifie "+infini" (hors de portée).
    private const double SensorNaNSentinel = 9.91e37;
    private const double SensorInfinitySentinel = 9.90e37;

    private static double? ParseNullableDouble(string? value)
    {
        return ParseSensorValue(value);
    }

    private static double? ParseDistanceKm(string? metersText)
    {
        if (string.IsNullOrWhiteSpace(metersText))
        {
            return null;
        }

        var trimmed = metersText.Trim();
        if (string.Equals(trimmed, "INF", StringComparison.OrdinalIgnoreCase))
        {
            return double.PositiveInfinity;
        }

        return ParseSensorValue(trimmed) is { } meters ? meters / 1000.0 : null;
    }

    private static double? ParseSensorValue(string? value)
    {
        if (!double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var result))
        {
            return null;
        }

        if (result == SensorNaNSentinel)
        {
            return double.NaN;
        }

        if (result == SensorInfinitySentinel)
        {
            return double.PositiveInfinity;
        }

        return result;
    }

    private async Task<string?> SendCommandAsync(string command, int timeoutMs, bool shouldLog = false)
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return null;
        }

        await _serialCommandGate.WaitAsync();
        try
        {
            var tcs = new TaskCompletionSource<string?>(TaskCreationOptions.RunContinuationsAsynchronously);
            lock (_serialSync)
            {
                _pendingResponse = tcs;
            }

            if (shouldLog)
            {
                AppendRaw($"-> {command}");
            }

            var port = _serialPort;
            await Task.Run(() => port.WriteLine(command));

            var completedTask = await Task.WhenAny(tcs.Task, Task.Delay(timeoutMs));
            if (completedTask != tcs.Task)
            {
                lock (_serialSync)
                {
                    if (_pendingResponse == tcs)
                    {
                        _pendingResponse = null;
                    }
                }

                AppendRaw($"! timeout sur {command}");
                return null;
            }

            var response = await tcs.Task;
            if (shouldLog && !string.IsNullOrWhiteSpace(response))
            {
                AppendRaw($"<- {response}");
            }

            return response;
        }
        finally
        {
            lock (_serialSync)
            {
                if (_pendingResponse is not null)
                {
                    _pendingResponse = null;
                }
            }

            _serialCommandGate.Release();
        }
    }

    private async Task<bool> SendWriteCommandAsync(string command, bool shouldLog = false)
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return false;
        }

        await _serialCommandGate.WaitAsync();
        try
        {
            if (shouldLog)
            {
                AppendRaw($"-> {command}");
            }

            var port = _serialPort;
            await Task.Run(() => port.WriteLine(command));
            return true;
        }
        finally
        {
            _serialCommandGate.Release();
        }
    }

    private void StartPollingTimer()
    {
        if (_isPollingPaused || _serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        StopPollingTimer();
        _pollingTimer = new DispatcherTimer(DispatcherPriority.Background, Dispatcher)
        {
            Interval = PollDeviceInterval
        };
        _pollingTimer.Tick += async (_, _) =>
        {
            try
            {
                await PollDeviceAsync();
            }
            catch (Exception ex)
            {
                AppendRaw($"Erreur de polling régulier : {ex.Message}");
            }
        };
        _pollingTimer.Start();
    }

    private void StopPollingTimer()
    {
        _pollingTimer?.Stop();
        _pollingTimer = null;
    }

    private void UpdateStatusUi()
    {
        Dispatcher.Invoke(() =>
        {
            if (_selfTestResult == "OK")
            {
                ModuleStatusText.Text = "Auto-test OK";
                StatusIndicator.Fill = Brushes.Green;
            }
            else
            {
                ModuleStatusText.Text = "Auto-test en erreur";
                StatusIndicator.Fill = Brushes.Red;
            }

            if (!string.IsNullOrWhiteSpace(_identity))
            {
                ModuleNameText.Text = _identity;
            }

            UpdateConfigurationUi();
        });
    }

    private void UpdateConfigurationUi()
    {
        Dispatcher.Invoke(() =>
        {
        });
    }

    private void ResetSessionState()
    {
        _pendingData.Clear();
        _manualCommandHistory.Clear();
        _manualCommandHistoryIndex = -1;
        _detections.Clear();
        _lastLightningCount = -1;
        _lastNoiseCount = -1;
        _lastDisturberCount = -1;
        _lastLightningDetectionTime = null;
        LightningLegendText.Text = FormatLightningCountText(_lastLightningCount);
        UpdateStormStatus();
        _currentAfe = "?";
        _currentNoiseThreshold = "?";
        _currentSpikeReject = "?";
        _currentDisturberState = "?";
        _currentLightningThreshold = "?";
        _selfTestResult = "Inconnu";
        _identity = string.Empty;
        _errorsSummary = string.Empty;
        RawOutputTextBox.Text = string.Empty;
        NoiseLevelFill.BeginAnimation(FrameworkElement.HeightProperty, null);
        NoiseLevelFill.Height = 0;
        DisturberLevelFill.BeginAnimation(FrameworkElement.HeightProperty, null);
        DisturberLevelFill.Height = 0;
        foreach (var dot in _chartDots)
        {
            LightningChartCanvas.Children.Remove(dot);
        }

        _chartDots.Clear();
        ResetStatusUi();
    }

    private void ResetStatusUi()
    {
        ModuleStatusText.Text = "Non connecté";
        StatusIndicator.Fill = Brushes.Gray;
        ModuleNameText.Text = "AS3935";
        RawOutputTextBox.Text = "Aucune donnée reçue pour le moment.";
    }

    private void AppendRaw(string message)
    {
        Dispatcher.Invoke(() =>
        {
            RawOutputTextBox.AppendText(message + Environment.NewLine);
            RawOutputTextBox.ScrollToEnd();
        });
    }

    private void UpdatePauseButtonState()
    {
        Dispatcher.Invoke(() =>
        {
            PauseResumeButton.IsEnabled = _serialPort is { IsOpen: true };
            PauseResumeButton.Content = _isPollingPaused ? "Reprendre" : "Pause";
        });
    }

    private void UpdateConnectionDependentUi()
    {
        var isConnected = _serialPort is { IsOpen: true };

        Dispatcher.Invoke(() =>
        {
            ConnectToggleButton.Content = isConnected ? "Déconnexion" : "Connexion";
            AfeComboBox.IsEnabled = isConnected;
            NoiseThresholdComboBox.IsEnabled = isConnected;
            WatchdogThresholdComboBox.IsEnabled = isConnected;
            SpikeRejectComboBox.IsEnabled = isConnected;
            DisturberStateCheckBox.IsEnabled = isConnected;
            LightningThresholdComboBox.IsEnabled = isConnected;
            ReloadConfigurationButton.IsEnabled = isConnected;
            ResetConfigurationButton.IsEnabled = isConnected;
            RecallConfigurationButton.IsEnabled = isConnected;
            SaveConfigurationButton.IsEnabled = isConnected;
            ConfigurationSlotComboBox.IsEnabled = isConnected;
            ManualCommandTextBox.IsEnabled = isConnected;
            ErrorQueryButton.IsEnabled = isConnected;
            ClearStatusButton.IsEnabled = isConnected;
            OperStatusButton.IsEnabled = isConnected;
            QuesStatusButton.IsEnabled = isConnected;
            SerialPollingIntervalComboBox.IsEnabled = true;
            ToggleSerialPollingButton.IsEnabled = true;
            UpdateSerialPollingButtonState();
            // Enable Read STB only when connected and serial polling is not active
            ReadStbButton.IsEnabled = isConnected && !_isSerialPollingEnabled;
            UpdateStbLeds(0);
            UpdateBargraphAvailability();
        });
    }

    private void SerialPollingIntervalComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        if (SerialPollingIntervalComboBox.SelectedItem is ComboBoxItem item && item.Content is string text)
        {
            _serialPollingInterval = text switch
            {
                "100ms" => TimeSpan.FromMilliseconds(100),
                "500ms" => TimeSpan.FromMilliseconds(500),
                "1s" => TimeSpan.FromSeconds(1),
                "2s" => TimeSpan.FromSeconds(2),
                "5s" => TimeSpan.FromSeconds(5),
                "10s" => TimeSpan.FromSeconds(10),
                _ => TimeSpan.FromSeconds(1)
            };

            if (_serialPollingTimer is not null && _isSerialPollingEnabled)
            {
                _serialPollingTimer.Stop();
                _serialPollingTimer.Interval = _serialPollingInterval;
                _serialPollingTimer.Start();
            }
        }
    }

    private async void ToggleSerialPollingButton_Click(object sender, RoutedEventArgs e)
    {
        if (_isSerialPollingEnabled)
        {
            _isSerialPollingEnabled = false;
            UpdateSerialPollingButtonState();
            StopSerialPollingTimer();
            return;
        }

        await EnableSerialPollingAsync();
    }

    private async Task EnableSerialPollingAsync()
    {
        _isSerialPollingEnabled = true;
        UpdateSerialPollingButtonState();
        StartSerialPollingTimer();

        if (_serialPort is { IsOpen: true })
        {
            await PollStatusByteAsync();
        }
    }

    private void UpdateSerialPollingButtonState()
    {
        Dispatcher.Invoke(() =>
        {
            ToggleSerialPollingButton.Content = _isSerialPollingEnabled ? "Désactiver" : "Activer";
            ToggleSerialPollingButton.IsEnabled = true;
            SerialPollingIntervalComboBox.IsEnabled = true;
            // Read STB should be disabled when automatic serial polling is active
            ReadStbButton.IsEnabled = _serialPort is { IsOpen: true } && !_isSerialPollingEnabled;
            ErrorQueryButton.IsEnabled = _serialPort is { IsOpen: true };
            ClearStatusButton.IsEnabled = _serialPort is { IsOpen: true };
            OperStatusButton.IsEnabled = _serialPort is { IsOpen: true };
            QuesStatusButton.IsEnabled = _serialPort is { IsOpen: true };
        });
    }

    private async void ReadStbButton_Click(object sender, RoutedEventArgs e)
    {
        if (_isSerialPollingEnabled)
        {
            return;
        }

        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        try
        {
            await PollStatusByteAsync();
        }
        catch (Exception ex)
        {
            AppendRaw($"Erreur lecture STB : {ex.Message}");
        }
    }

    private void StartSerialPollingTimer()
    {
        StopSerialPollingTimer();
        if (!_isSerialPollingEnabled)
        {
            return;
        }

        _serialPollingTimer = new DispatcherTimer(DispatcherPriority.Background, Dispatcher)
        {
            Interval = _serialPollingInterval
        };
        _serialPollingTimer.Tick += async (_, _) =>
        {
            if (_serialPort is { IsOpen: true })
            {
                await PollStatusByteAsync();
            }
        };
        _serialPollingTimer.Start();
    }

    private void StopSerialPollingTimer()
    {
        _serialPollingTimer?.Stop();
        _serialPollingTimer = null;
    }

    private async Task PollStatusByteAsync()
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            UpdateStbLeds(0);
            return;
        }

        await _serialCommandGate.WaitAsync();
        try
        {
            _isReadingStb = true;
            try
            {
                var port = _serialPort;
                await Task.Run(() =>
                {
                    port.DiscardInBuffer();
                    port.DiscardOutBuffer();
                    port.Write([0x98], 0, 1);
                    port.BaseStream.Flush();
                });

                var startByte = await ReadSerialByteAsync(200);
                if (startByte != 0x18)
                {
                    AppendRaw("! lecture STB : octet de départ invalide");
                    UpdateStbLeds(0);
                    return;
                }

                var stbByte = await ReadSerialByteAsync(100);
                var endByte = await ReadSerialByteAsync(100);
                if (endByte != 0x19)
                {
                    AppendRaw("! lecture STB : octet de fin invalide");
                    UpdateStbLeds(0);
                    return;
                }

                var stbValue = stbByte & 0xFF;
                _lastStbValue = stbValue;
                UpdateStbLeds(stbValue);
            }
            finally
            {
                _isReadingStb = false;
            }
        }
        catch (TimeoutException tex)
        {
            AppendRaw($"! timeout lors de la lecture STB : {tex.Message}");
            UpdateStbLeds(0);
        }
        catch (Exception ex)
        {
            AppendRaw($"! erreur lors de la lecture STB : {ex.Message}");
            UpdateStbLeds(0);
        }
        finally
        {
            _serialCommandGate.Release();
        }
    }

    private async Task<int> ReadSerialByteAsync(int timeoutMs)
    {
        var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
        while (DateTime.UtcNow < deadline)
        {
            if (_serialPort is null || !_serialPort.IsOpen)
            {
                throw new TimeoutException("Le port série n'est plus ouvert.");
            }

            if (_serialPort.BytesToRead > 0)
            {
                return _serialPort.ReadByte();
            }

            await Task.Delay(10);
        }

        throw new TimeoutException("Temps d'attente dépassé pour le polling STB.");
    }

    private void UpdateStbLeds(int stbValue)
    {
        Dispatcher.Invoke(() =>
        {
            var bits = new[]
            {
                (StbBit0Led, 0x01, Brushes.LightGreen),
                (StbBit1Led, 0x02, Brushes.LightGreen),
                (StbBit2Led, 0x04, Brushes.Red),
                (StbBit3Led, 0x08, Brushes.Gold),
                (StbBit4Led, 0x10, Brushes.Gold),
                (StbBit5Led, 0x20, Brushes.Orange),
                (StbBit6Led, 0x40, Brushes.Blue),
                (StbBit7Led, 0x80, Brushes.Orange)
            };

            foreach (var (ellipse, mask, color) in bits)
            {
                ellipse.Fill = (stbValue & mask) != 0 ? color : Brushes.LightGray;
            }
        });
    }

    private void ResetSerialPollingState()
    {
        _isSerialPollingEnabled = false;
        _serialPollingInterval = TimeSpan.FromSeconds(1);
        StopSerialPollingTimer();
        UpdateStbLeds(0);
        UpdateSerialPollingButtonState();
    }

    private async void ConfigurationComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        if (_suppressConfigurationUpdates || _isApplyingConfiguration || _serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        await ApplyChangedConfigurationAsync(sender as FrameworkElement);
    }

    private async void ConfigurationCheckBox_Checked(object sender, RoutedEventArgs e)
    {
        UpdateBargraphAvailability();

        if (_suppressConfigurationUpdates || _isApplyingConfiguration || _serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        await ApplyChangedConfigurationAsync(sender as FrameworkElement);
    }

    private void UpdateBargraphAvailability()
    {
        var isConnected = _serialPort is { IsOpen: true };

        NoiseUnavailableText.Visibility = isConnected ? Visibility.Collapsed : Visibility.Visible;
        if (!isConnected)
        {
            _lastNoiseCount = -1;
            NoiseLevelFill.BeginAnimation(FrameworkElement.HeightProperty, null);
            NoiseLevelFill.Height = 0;
        }

        var isDisturberAvailable = isConnected && DisturberStateCheckBox.IsChecked == true;
        DisturberUnavailableText.Visibility = isDisturberAvailable ? Visibility.Collapsed : Visibility.Visible;
        if (!isDisturberAvailable)
        {
            _lastDisturberCount = -1;
            DisturberLevelFill.BeginAnimation(FrameworkElement.HeightProperty, null);
            DisturberLevelFill.Height = 0;
        }
    }

    private async Task ApplyChangedConfigurationAsync(FrameworkElement? changedControl)
    {
        if (_isApplyingConfiguration)
        {
            return;
        }

        _isApplyingConfiguration = true;
        try
        {
            if (changedControl == AfeComboBox)
            {
                var afeValue = GetSelectedComboBoxValue(AfeComboBox);
                var appliedValue = await ApplyConfigurationValueAsync(":SENSe:AFE:GAIN", afeValue switch
                {
                    "intérieur" => "INDoor",
                    "extérieur" => "OUTdoor",
                    _ => "INDoor"
                });
                await ApplyConfigurationValueToUiAsync(":SENSe:AFE:GAIN", appliedValue);
                return;
            }

            if (changedControl == NoiseThresholdComboBox)
            {
                var noiseValue = GetSelectedComboBoxValue(NoiseThresholdComboBox);
                var appliedValue = await ApplyConfigurationValueAsync(":SENSe:NOISe:THReshold", noiseValue);
                await ApplyConfigurationValueToUiAsync(":SENSe:NOISe:THReshold", appliedValue);
                return;
            }

            if (changedControl == SpikeRejectComboBox)
            {
                var rejectValue = GetSelectedComboBoxValue(SpikeRejectComboBox);
                var appliedValue = await ApplyConfigurationValueAsync(":SENSe:SPIKe:REJection", rejectValue);
                await ApplyConfigurationValueToUiAsync(":SENSe:SPIKe:REJection", appliedValue);
                return;
            }

            if (changedControl == WatchdogThresholdComboBox)
            {
                var watchdogValue = GetSelectedComboBoxValue(WatchdogThresholdComboBox);
                var appliedValue = await ApplyConfigurationValueAsync(":SENSe:WATChdog:THReshold", watchdogValue);
                await ApplyConfigurationValueToUiAsync(":SENSe:WATChdog:THReshold", appliedValue);
                return;
            }

            if (changedControl == DisturberStateCheckBox)
            {
                var stateValue = DisturberStateCheckBox.IsChecked == true ? "1" : "0";
                var appliedValue = await ApplyConfigurationValueAsync(":SYSTem:DISTurber:STATe", stateValue);
                await ApplyConfigurationValueToUiAsync(":SYSTem:DISTurber:STATe", appliedValue);
                return;
            }

            if (changedControl == LightningThresholdComboBox)
            {
                var lightningValue = GetSelectedComboBoxValue(LightningThresholdComboBox);
                var appliedValue = await ApplyConfigurationValueAsync(":SENSe:LIGHtning:THReshold", lightningValue);
                await ApplyConfigurationValueToUiAsync(":SENSe:LIGHtning:THReshold", appliedValue);
            }
        }
        finally
        {
            _isApplyingConfiguration = false;
        }
    }

    private async Task<string?> ApplyConfigurationValueAsync(string commandBase, string value)
    {
        var command = $"{commandBase} {value}";
        var writeSucceeded = await SendWriteCommandAsync(command);
        if (!writeSucceeded)
        {
            AppendRaw($"! échec d'envoi de {command}");
            return null;
        }

        var verifyCommand = commandBase.EndsWith("?", StringComparison.Ordinal) ? commandBase : commandBase + "?";
        var verifyResponse = await SendCommandAsync(verifyCommand, 2000);
        if (string.IsNullOrWhiteSpace(verifyResponse))
        {
            AppendRaw($"! vérification impossible pour {command}");
            return null;
        }

        var normalizedValue = NormalizeVerificationValue(commandBase, verifyResponse);
        var expectedValue = NormalizeVerificationValue(commandBase, value);
        if (!string.Equals(normalizedValue, expectedValue, StringComparison.OrdinalIgnoreCase))
        {
            AppendRaw($"! valeur non confirmée pour {command}: attendu {expectedValue}, reçu {normalizedValue}");
            return normalizedValue;
        }

        AppendRaw($"✓ {command} appliquée");
        return normalizedValue;
    }

    private async Task ApplyConfigurationValueToUiAsync(string commandBase, string? value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return;
        }

        await Dispatcher.InvokeAsync(() =>
        {
            switch (commandBase)
            {
                case ":SENSe:AFE:GAIN":
                    _currentAfe = NormalizeAfeResponse(value);
                    SelectComboBoxValue(AfeComboBox, _currentAfe switch
                    {
                        "IND" => "intérieur",
                        "OUT" => "extérieur",
                        _ => _currentAfe
                    });
                    break;
                case ":SENSe:NOISe:THReshold":
                    _currentNoiseThreshold = NormalizeValue(value);
                    SelectComboBoxValue(NoiseThresholdComboBox, _currentNoiseThreshold);
                    break;
                case ":SENSe:WATChdog:THReshold":
                    _currentWatchdogThreshold = NormalizeValue(value);
                    SelectComboBoxValue(WatchdogThresholdComboBox, _currentWatchdogThreshold);
                    break;
                case ":SENSe:SPIKe:REJection":
                    _currentSpikeReject = NormalizeValue(value);
                    SelectComboBoxValue(SpikeRejectComboBox, _currentSpikeReject);
                    break;
                case ":SYSTem:DISTurber:STATe":
                    _currentDisturberState = NormalizeBooleanResponse(value);
                    DisturberStateCheckBox.IsChecked = _currentDisturberState != "0";
                    break;
                case ":SENSe:LIGHtning:THReshold":
                    _currentLightningThreshold = NormalizeValue(value);
                    SelectComboBoxValue(LightningThresholdComboBox, _currentLightningThreshold);
                    break;
            }

            UpdateConfigurationUi();
        });
    }

    private static string NormalizeVerificationValue(string commandBase, string? value)
    {
        var normalized = NormalizeValue(value);

        return commandBase switch
        {
            ":SENSe:AFE:GAIN" => normalized.ToUpperInvariant() switch
            {
                "INDOOR" => "IND",
                "OUTDOOR" => "OUT",
                _ => normalized
            },
            ":SYSTem:DISTurber:STATe" => normalized switch
            {
                "1" => "1",
                "0" => "0",
                _ => normalized
            },
            _ => normalized
        };
    }

    private static string GetSelectedComboBoxValue(ComboBox comboBox)
    {
        return comboBox.SelectedItem is ComboBoxItem item ? item.Content?.ToString() ?? string.Empty : string.Empty;
    }

    private void PauseResumeButton_Click(object sender, RoutedEventArgs e)
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        _isPollingPaused = !_isPollingPaused;
        if (_isPollingPaused)
        {
            StopPollingTimer();
            AppendRaw("Interrogation régulière mise en pause.");
        }
        else
        {
            StartPollingTimer();
            AppendRaw("Interrogation régulière reprise.");
        }

        UpdatePauseButtonState();
    }

    private async void ReloadConfigurationButton_Click(object sender, RoutedEventArgs e)
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        await LoadConfigurationAsync();
    }

    private async void ResetConfigurationButton_Click(object sender, RoutedEventArgs e)
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        var resetSent = await SendWriteCommandAsync("*RST");
        if (!resetSent)
        {
            AppendRaw("! échec d'envoi de *RST");
            return;
        }

        await LoadConfigurationAsync();
    }

    private async void RecallConfigurationButton_Click(object sender, RoutedEventArgs e)
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        var slot = GetSelectedComboBoxValue(ConfigurationSlotComboBox);
        if (string.IsNullOrWhiteSpace(slot))
        {
            return;
        }

        var recallSent = await SendWriteCommandAsync($"*RCL {slot}");
        if (!recallSent)
        {
            AppendRaw($"! échec d'envoi de *RCL {slot}");
            return;
        }

        await LoadConfigurationAsync();
    }

    private async void SaveConfigurationButton_Click(object sender, RoutedEventArgs e)
    {
        if (_serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        var slot = GetSelectedComboBoxValue(ConfigurationSlotComboBox);
        if (string.IsNullOrWhiteSpace(slot))
        {
            return;
        }

        var saveSent = await SendWriteCommandAsync($"*SAV {slot}");
        if (!saveSent)
        {
            AppendRaw($"! échec d'envoi de *SAV {slot}");
            return;
        }
    }

    private static bool ExpectsResponse(string command)
    {
        return command.Contains('?', StringComparison.Ordinal);
    }

    private static bool IsValidIdentity(string identity)
    {
        var fields = identity.Split(',', StringSplitOptions.TrimEntries);
        return fields.Length >= 2
            && string.Equals(fields[0], "ValTronix", StringComparison.OrdinalIgnoreCase)
            && string.Equals(fields[1], "LightningDetector", StringComparison.OrdinalIgnoreCase);
    }

    private static string NormalizeAfeResponse(string? value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return "?";
        }

        return value.Trim().ToUpperInvariant() switch
        {
            "IND" => "IND",
            "OUT" => "OUT",
            _ => value.Trim()
        };
    }

    private static string NormalizeValue(string? value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return "?";
        }

        var trimmed = value.Trim();
        return trimmed.StartsWith('+') ? trimmed[1..] : trimmed;
    }

    private static string NormalizeBooleanResponse(string? value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return "0";
        }

        var trimmed = value.Trim();
        if (trimmed.StartsWith('+'))
        {
            trimmed = trimmed[1..];
        }

        return trimmed switch
        {
            "1" => "1",
            "0" => "0",
            _ => "0"
        };
    }

    private static string FormatErrors(string? errors)
    {
        if (string.IsNullOrWhiteSpace(errors))
        {
            return "Aucune erreur";
        }

        return errors.Trim();
    }

    private void SelectComboBoxValue(ComboBox comboBox, string? value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return;
        }

        foreach (ComboBoxItem item in comboBox.Items)
        {
            if (item.Content?.ToString()?.Equals(value, StringComparison.OrdinalIgnoreCase) == true)
            {
                comboBox.SelectedItem = item;
                return;
            }
        }

        if (comboBox.Items.Count > 0)
        {
            comboBox.SelectedIndex = 0;
        }
    }

    private void ManualCommandTextBox_TextChanged(object sender, TextChangedEventArgs e)
    {
        if (_isApplyingHistorySelection)
        {
            return;
        }

        _manualCommandHistoryIndex = -1;
        _manualCommandHistoryPrefix = null;
    }

    private void ApplyHistorySelection(string command)
    {
        _isApplyingHistorySelection = true;
        try
        {
            ManualCommandTextBox.Text = command;
            ManualCommandTextBox.Select(ManualCommandTextBox.Text.Length, 0);
        }
        finally
        {
            _isApplyingHistorySelection = false;
        }
    }

    private string GetPrefixBeforeCursor()
    {
        var caretIndex = Math.Max(0, Math.Min(ManualCommandTextBox.Text?.Length ?? 0, ManualCommandTextBox.SelectionStart));
        return ManualCommandTextBox.Text?.Substring(0, caretIndex) ?? string.Empty;
    }

    private bool TryNavigateHistory(bool moveUp)
    {
        if (_manualCommandHistory.Count == 0)
        {
            return false;
        }

        string prefix;
        if (_manualCommandHistoryIndex < 0)
        {
            // Début d'une nouvelle session de navigation : on fige le préfixe courant
            // (même s'il est vide), pour ne pas le recalculer après chaque sélection
            // (le curseur se retrouve en fin de texte après ApplyHistorySelection).
            prefix = GetPrefixBeforeCursor();
            _manualCommandHistoryPrefix = prefix;
        }
        else
        {
            prefix = _manualCommandHistoryPrefix ?? string.Empty;
        }

        var startIndex = moveUp
            ? (_manualCommandHistoryIndex >= 0 ? _manualCommandHistoryIndex - 1 : _manualCommandHistory.Count - 1)
            : (_manualCommandHistoryIndex >= 0 ? _manualCommandHistoryIndex + 1 : -1);

        if (moveUp)
        {
            for (var index = startIndex; index >= 0; index--)
            {
                var candidate = _manualCommandHistory[index];
                if (string.IsNullOrEmpty(prefix) || candidate.StartsWith(prefix, StringComparison.Ordinal))
                {
                    _manualCommandHistoryIndex = index;
                    ApplyHistorySelection(candidate);
                    return true;
                }
            }

            return false;
        }

        for (var index = startIndex; index < _manualCommandHistory.Count; index++)
        {
            var candidate = _manualCommandHistory[index];
            if (string.IsNullOrEmpty(prefix) || candidate.StartsWith(prefix, StringComparison.Ordinal))
            {
                _manualCommandHistoryIndex = index;
                ApplyHistorySelection(candidate);
                return true;
            }
        }

        return false;
    }

    private void ManualCommandTextBox_PreviewKeyDown(object sender, KeyEventArgs e)
    {
        if (e.Key == Key.Up)
        {
            if (TryNavigateHistory(moveUp: true))
            {
                e.Handled = true;
            }

            return;
        }

        if (e.Key == Key.Down)
        {
            if (TryNavigateHistory(moveUp: false))
            {
                e.Handled = true;
            }
        }
    }

    private async void ManualCommandTextBox_KeyDown(object sender, KeyEventArgs e)
    {
        if (e.Key != Key.Enter)
        {
            return;
        }

        var command = ManualCommandTextBox.Text?.Trim();
        if (string.IsNullOrWhiteSpace(command))
        {
            return;
        }

        e.Handled = true;
        await ExecuteManualCommandAsync(command);
    }

    private async void ManualQuickCommandButton_Click(object sender, RoutedEventArgs e)
    {
        if (sender is not Button button)
        {
            return;
        }

        var command = button.Tag?.ToString();
        if (string.IsNullOrWhiteSpace(command))
        {
            return;
        }

        await ExecuteManualCommandAsync(command);
    }

    private async Task ExecuteManualCommandAsync(string command)
    {
        if (string.IsNullOrWhiteSpace(command) || _serialPort is null || !_serialPort.IsOpen)
        {
            return;
        }

        if (_manualCommandHistory.Count == 0 || _manualCommandHistory[^1] != command)
        {
            _manualCommandHistory.Add(command);
            if (_manualCommandHistory.Count > 100)
            {
                _manualCommandHistory.RemoveAt(0);
            }
        }

        _manualCommandHistoryIndex = -1;
        _manualCommandHistoryPrefix = null;
        ManualCommandTextBox.Clear();

        try
        {
            if (ExpectsResponse(command))
            {
                await SendCommandAsync(command, 2000, shouldLog: true);
            }
            else
            {
                var writeSucceeded = await SendWriteCommandAsync(command, shouldLog: true);
                if (!writeSucceeded)
                {
                    AppendRaw($"! échec d'envoi de {command}");
                    return;
                }

                _errorsSummary = "Aucune erreur";
                UpdateConfigurationUi();
            }
        }
        catch (Exception ex)
        {
            AppendRaw($"Erreur de commande manuelle : {ex.Message}");
        }
    }

    public class DetectionRow
    {
        public DateTime Timestamp { get; init; }
        public double? DistanceKm { get; init; }
        public double? Energy { get; init; }

        public string TimestampText => Timestamp.ToString("yyyy-MM-dd HH:mm:ss");

        public string DistanceKmText
        {
            get
            {
                if (DistanceKm is not { } km || double.IsNaN(km))
                {
                    return "NAN";
                }

                return double.IsPositiveInfinity(km) ? "∞" : km.ToString("0.###", CultureInfo.InvariantCulture);
            }
        }

        public string EnergyText => Energy is { } value ? value.ToString(CultureInfo.InvariantCulture) : "NAN";
    }
}
