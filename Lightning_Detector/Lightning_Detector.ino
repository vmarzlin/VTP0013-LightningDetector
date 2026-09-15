/*
 * Détecteur de foudre à base d'AS3935, compatible SCPI.
 *
 * =====================================================================
 *                    BROCHAGE ARDUINO NANO (ATmega328P)
 * =====================================================================
 *
 * Vue de dessus (USB en bas) :
 *
 *       D1/TX  [1] +-----+ [30]  VIN
 *       D0/RX  [2] | ::: | [29]  GND
 *      RESET   [3] |     | [28]  RESET
 *        GND   [4] |     | [27]  +5V
 *  INT0 / D2   [5] |     | [26]  A7
 *  INT1 / D3~  [6] |     | [25]  A6
 *         D4   [7] |     | [24]  A5 / SCL
 *         D5~  [8] |     | [23]  A4 / SDA
 *         D6~  [9] |     | [22]  A3
 *         D7  [10] |     | [21]  A2
 *         D8  [11] |     | [20]  A1
 *         D9~ [12] |     | [19]  A0
 *  !SS / D10~ [13] |     | [18]  AREF
 * MOSI / D11~ [14] |     | [17]  3V3
 * MISO / D12  [15] +-***-+ [16]  D13 / SCK / LED
 *
 * ---------------------------------------------------------------------
 * Broches analogiques (A0-A7) :
 *   A0-A5 : Analogiques + Digitales (A4 = SDA, A5 = SCL)
 *   A6-A7 : Analogiques uniquement (pas d'entrée/sortie digitale)
 *
 * Alimentation :
 *   VIN  : 7-12 V (entrée non régulée)
 *   +5V  : Sortie / entrée 5 V
 *   3V3  : Sortie 3,3 V (courant limité)
 *   GND  : Masse
 *   AREF : Référence analogique
 * =====================================================================
 */
#include <SPI.h>
#include <EEPROM.h>
#include <RTC.h>
#include "AS3935.h"

// #define DEBUG_PARSING
// #define BUZZER_OC2B
// #define SCPI_MAP
// #define SCPI_PUD
#define SCPI_PSC
// #define FIRMWARE_VERSION "1.0.0"

#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 32
#endif

enum StatFlag : uint8_t {
  STAT_NOISE = 0x01,
  STAT_DISTURBER = 0x02,
  STAT_DISTANCE = 0x20,
  STAT_ENERGY = 0x40,
  STAT_LIGHTNING = 0x80,
  STAT_ALL = 0xFF
};

// Interrupt pin for lightning detection 
const int lightningInt = 4; // changer vers 2 (INT0) ?
// Chip select pin
const int spiCS = 10;

// Pin LED & Buzzer
const int lightningLed = 9;
const int noiseLed = 6;
const int disturbLed = 5;
const int passiveBuzzer = 3;
const int errorLed = 8;
const int calBtn = A0;

/*
  EEPROM memory map (1ko: 0x0000 - 0x03FF)

 0000-000F : Serial Number (8 bytes) !Serial Number (8 bytes)
 0010-001F : CONF_BASE
 0020-002F
 0030-003F
 0040-004F
 0080-008F
 0090-009F
 ...
 0100-010F : CALI_BASE
 ...
 0170-017F
 ...
 0200-020F : SLOT_BASE (*SAV / *RCL)
 ...
 03F0-03FF
 END OF EEPROM
*/

// REG0x00[5:1]
#define INDOOR 0x12 
#define OUTDOOR 0xE
bool isAs3935Available;
const int noiseMinimum = 0;       // REG0x01[6:4] = NF_LEV
const int noiseDefault = 2;
const int noiseMaximum = 7;
const int wdthresMinimum = 0;     // REG0x01[3:0] = WDTH
const int wdthresDefault = 2;
const int wdthresMaximum = 10;
const int lightningMinimum = 1;   // REG0x02[5:4] = MIN_NUM_LIGH
const int lightningDefault = 1;
const int lightningMaximum = 16;
const int spikeMinimum = 0;       // REG0x02[3:0] = SREJ
const int spikeDefault = 2;
const int spikeMaximum = 15;
const bool disturbDefault = true; // REG0x03[5] = MASK_DIST (disturberDefault indique qu'on veut aussi les parasites, donc que par défaut ils ne sont PAS masqués)
bool isDisturberMasked = false;
bool isProtected = true;

const int buzzerFreqMinimum = 32;
const int buzzerFreqDefault = 1000;
const int buzzerFreqMaximum = 6000;
const float buzzerDuraMinimum = 0.100;
const float buzzerDuraDefault = 0.250;
const float buzzerDuraMaximum = 5.000;
const bool buzzerStateDefault = true;

#define VCC_MIN_MV 2400
#define VCC_MAX_MV 5500

#define SCPI_MAX_ARGS 8
#define SCPI_BUFFER_SIZE 128
char scpiBuffer[SCPI_BUFFER_SIZE] = {0};
uint8_t scpiBufferIndex;
int16_t scpiError;
uint8_t scpiEse, scpiEsr, scpiSre, scpiStb;
#ifdef SCPI_MAP
int16_t scpiQuesMap[15] = {0};
int16_t scpiOperMap[15] = {0};
#endif
struct ScpiRegister {
  uint16_t con;   // CONDition
  uint16_t eve;   // EVENt
  uint16_t ena;   // ENABle (bit 15 = defaut)
  uint16_t ptr;   // PTRansition
  uint16_t ntr;   // NTRansition
#ifdef SCPI_MAP
  int16_t *map;   // MAP[15] ou nullptr si non utilisé (cas LIGHtning)
#endif
};
ScpiRegister quesReg; // = {0, 0, 0x0000, 0x7FFF, 0x0000, scpiQuesMap};
ScpiRegister operReg; // = {0, 0, 0x0000, 0x7FFF, 0x0000, scpiOperMap};
ScpiRegister voltReg; // = {0, 0, 0xFFFF, 0x7FFF, 0x0000, nullptr}; // pas de MAP
ScpiRegister timeReg; // = {0, 0, 0xFFFF, 0x7FFF, 0x0000, nullptr}; // pas de MAP
ScpiRegister tempReg; // = {0, 0, 0xFFFF, 0x7FFF, 0x0000, nullptr}; // pas de MAP
ScpiRegister caliReg; // = {0, 0, 0xFFFF, 0x7FFF, 0x0000, nullptr}; // pas de MAP
ScpiRegister lighReg; // = {0, 0, 0xFFFF, 0x7FFF, 0x0000, nullptr}; // pas de MAP

enum : uint8_t {
  OPER_CAL = 0,   // CALibrating — The instrument is currently performing a calibration.
  OPER_SETT = 1,  // SETTling — The instrument is waiting for signals it controls to stabilize enough to begin measurements.
  OPER_RANG = 2,  // RANGing — The instrument is currently changing its range.
  OPER_SWE = 3,   // SWEeping — A sweep is in progress.
  OPER_MEAS = 4,  // MEASuring — The instrument is actively measuring.
  OPER_TRIG = 5,  // Waiting for TRIG — The instrument is in a “wait for trigger” state of the trigger model.
  OPER_ARM = 6,   // Waiting for ARM — The instrument is in a “wait for arm” state of the trigger model.
  OPER_CORR = 7,  // CORRecting — The instrument is currently performing a correction.
  // 8-12: available to designer
  OPER_INST = 13, // INSTrument Summary Bit One of n multiple logical instruments is reporting OPERational status.
  OPER_PROG = 14  // PROGram running — A user-defined programming is currently in the run state.
  // 15: always 0
};

enum : uint8_t {
  QUES_VOLT = 0,  // Summary of VOLTage
  QUES_CURR = 1,  // Summary of CURRent
  QUES_TIME = 2,  // Summary of TIME
  QUES_POW  = 3,  // Summary of POWer
  QUES_TEMP = 4,  // Summary of TEMPerature
  QUES_FREQ = 5,  // Summary of FREQuency
  QUES_PHAS = 6,  // Summary of PHASe
  QUES_MOD  = 7,  // Summary of MODulation
  QUES_CAL  = 8,  // Summary of CALibration
  QUES_WARN = 9,  // Summary of WARNing
  QUES_LIGH = 10, // Summary of LIGHtning
  // 11-12: Available to designer
  QUES_INST = 13, // INSTrument Summary
  QUES_CMDW = 14  // Command Warning
  // 15: always 0
};

enum : uint8_t {
  VOLT_VCOV = 0,  // VCC over voltage
  VOLT_VCUV = 1,  // VCC under voltage
  // 2-3: Reserved for VDD (3.3V)
  VOLT_BATL = 4   // Backup battery low
  // 5-14: Free
  // 15: always 0
};

enum : uint8_t {
  TIME_NTST = 0,  // Time not set
  TIME_NTAV = 1,  // RTC not available
  TIME_BATL = 2,  // RTC battery low
  TIME_EXNA = 3   // External time reference not available
  // 4-14: Free
  // 15: always 0
};

enum : uint8_t {
  TEMP_OTP = 0,   // over-temperature
  TEMP_UTP = 1,   // under temperature
  TEMP_FAUL = 2,  // sensor failure
  TEMP_NCAL = 3   // sensor not calibrated
  // 4-14: Free
  // 15: always 0
};

enum : uint8_t {
  CALI_NOTR = 0,  // TRCO not calibrated
  CALI_TRCO = 1,  // autocalibration TRCO (Timer RC Oscillator) failed
  CALI_NOSR = 2,  // SRCO not calibrated
  CALI_SRCO = 3,  // autocalibration SRCO (System RC Oscillator) failed
  CALI_FAIL = 4,  // other autocalibration failed
  // 5-12: Free
  CALI_EXPI = 13, // calibration expired
  CALI_CDCO = 14  // calibration data corrupted
  // 15: always 0
};

enum : uint8_t {
  LIGH_LIGH = 0,  // Lightning strike detected !
  LIGH_DIST = 1,  // Disturber detected
  LIGH_SAPR = 2,  // Storm approaching
  LIGH_SLEA = 3,  // Storm leaving
  LIGH_STOR = 4,  // Storm in progress
  LIGH_SCLO = 5,  // Storm close
  // 6-13: Free
  LIGH_NOIS = 14  // Too noisy
  // 15: always 0
};

bool scpiOutput;
#ifdef SCPI_PSC
bool isPSC;
#endif

// Floating-point numbers can be as large as 3.4028235E+38 and as low as -3.4028235E+38.
// They are stored as 32 bits (4 bytes) of information.
const float SCPI_PINFINITY =  9.9E37;
const float SCPI_NINFINITY = -9.9E37;
const float SCPI_NAN       =  9.91E37;

constexpr size_t roundUpToMultiple(size_t value, size_t multiple)
{
  return ((value + multiple - 1) / multiple) * multiple;
}

/*****************************************************************************
 *                                                                           *
 *               Structures en EEPROM (ATmega328 => 1ko EEPROM)              *
 *                                                                           *
 *****************************************************************************/
/* 1ko : adresses de 0x000 à 0x3FF                                           */
// Paramètres non modifiables par l'utilisateur
#define PARA_BASE 0x000
#define PARA_VERSION 1
struct SavedParameters
{ // 8 bytes
  uint8_t version;
  uint8_t checksum;
  uint16_t x;
  uint32_t serial;
  uint16_t freqMin;
  uint16_t freqMax;
  uint16_t duraMin;
  uint16_t duraMax;
};
const uint16_t FREQ_MIN = 400;        // Hz (son grave pour orage lointain)
const uint16_t FREQ_MAX = 2500;       // Hz (son aigu pour orage proche)
const uint16_t DUREE_MIN = 50;        // ms (impact faible)
const uint16_t DUREE_MAX = 500;       // ms (impact puissant)

#ifdef SCPI_PSC
// Configuration chargée (ou non) à l'allumage (chargée si *PSC = OFF)
#define CONF_BASE 0x080
#define CONF_VERSION 1
struct SavedPonStatus
{ // 18 bytes
  uint8_t version;
  uint8_t checksum;
  uint8_t ese;
  uint8_t sre;
  uint16_t oper;
  uint16_t ques;
  uint16_t volt;
  uint16_t time;
  uint16_t temp;
  uint16_t cali;
  uint16_t ligh;
};
#endif

// Réglages utilisateurs (*SAV / *RCL)
#define SLOT_BASE 0x180
#define SLOT_VERSION 1
#define SLOT_MAGIC 0xA5
#define SLOT_COUNT 8  // 0 à 7 emplacements de sauvegarde de conf. (*SAV / *RCL)
struct SavedState
{ // 12 bytes
  uint8_t magic;
  uint8_t checksum;
  uint8_t version;          // Version 1
  // AS3935
  uint8_t afeMd;            // AFE_GB + MASK_DSIT (bit 7)
  uint8_t noiseFloor;       // NF_LEV
  uint8_t watchDogVal;      // WDTH
  uint8_t spike;            // SREJ
  uint8_t lightningThresh;  // MIN_NUM_LIGH
  // Conf interne
  uint16_t buzzerFreq;
  uint16_t buzzerDura;      // BIT 15 = buzzerEnabled
};
constexpr size_t SLOT_SIZE = roundUpToMultiple(sizeof(SavedState), 8);

/*****************************************************************************
 *                                                                           *
 *               Gestion de la file FIFO des erreurs/événements              *
 *                                                                           *
 *****************************************************************************/

#define ERROR_QUEUE_SIZE 8  // Taille de la FIFO (8 erreurs max)
int16_t errorQueue[ERROR_QUEUE_SIZE];
uint8_t queueHead = 0; // Indice d'écriture
uint8_t queueTail = 0; // Indice de lecture
uint8_t queueCount = 0; // Nombre d'erreurs en attente
/**
 * @brief Ajoute une erreur dans la FIFO d'erreurs SCPI, émet un bip et met à jour les registres d'état associés.
 * @param error Code d'erreur SCPI (valeur négative) à empiler.
 */
void pushError(int16_t error)
{
  systemBeep();
  if (queueCount >= ERROR_QUEUE_SIZE)
  {
    // Remplacer le dernier élément par "Queue overflow" (-350)
    uint8_t lastIndex = (queueHead == 0) ? (ERROR_QUEUE_SIZE - 1) : (queueHead - 1);
    errorQueue[lastIndex] = -350;
  }
  else
  {
    errorQueue[queueHead] = error;
    queueHead++;
    queueHead %= ERROR_QUEUE_SIZE;
    queueCount++;
  }

  // Mettre à jour le registre ESR selon la catégorie d'erreur (-100 à -499)
  // Le registre ESR est mis à jour même si la file FIFO est pleine.
  if ((error >= -499) && (error <= -100))
  {
    uint8_t category = (-error) / 100;
    bitSet(scpiEsr, 6 - category);
  }

#ifdef SCPI_MAP
  // Mettre à jour les registres OPERation et QUEStionable si un mapping correspond
  for (uint8_t i = 0; i < 15; i++)
  {
    if (scpiOperMap[i] == error)
    {
      regBitSet(operReg, i);
    }
    if (scpiQuesMap[i] == error)
    {
      regBitSet(quesReg, i);
    }
  }
  updateOperQues();
#endif
  updateStb();
}

/**
 * @brief Dépile et retourne la plus ancienne erreur de la FIFO (réponse à SYSTem:ERRor?).
 * @return Code d'erreur dépilé, ou 0 ("No error") si la file est vide.
 */
int16_t popError()
{
  if (queueCount == 0)
  {
    return 0; // 0 = "No error"
  }

  int16_t error = errorQueue[queueTail];
  queueTail++;
  queueTail %= ERROR_QUEUE_SIZE;
  queueCount--;
#ifdef SCPI_MAP
  updateOperQues();
#endif
  updateStb();
  return error;
}

/**
 * @brief Vide complètement la FIFO des erreurs (utilisé par la commande *CLS).
 */
void clearErrorQueue()
{
  queueHead = 0;
  queueTail = 0;
  queueCount = 0;
#ifdef SCPI_MAP
  updateOperQues();
#endif
  updateStb();
}

#ifdef SCPI_MAP
/**
 * @brief Met à jour les registres OPERation et QUEStionable selon le mapping des erreurs actuellement présentes dans la FIFO.
 */
void updateOperQues()
{
  // Mettre à jour les registres OPERation et QUEStionable si un mapping correspond
  for (uint8_t bit = 0; bit < 15; bit++)
  {
    if (scpiOperMap[bit] != 0)
    {
      bool found = false;
      uint8_t j = queueTail;
      for (uint8_t k = 0; k < queueCount; k++)
      {
        if (scpiOperMap[bit] == errorQueue[j]) found = true;
        j = (j + 1) % ERROR_QUEUE_SIZE;
      }
      if (!found) regBitClear(operReg, bit);
    }
    if (scpiQuesMap[bit] != 0)
    {
      bool found = false;
      uint8_t j = queueTail;
      for (uint8_t k = 0; k < queueCount; k++)
      {
        if (scpiQuesMap[bit] == errorQueue[j]) found = true;
        j = (j + 1) % ERROR_QUEUE_SIZE;
      }
      if (!found) regBitClear(quesReg, bit);
    }
  }
}
#endif

#ifndef FIRMWARE_VERSION
// Table des mois, en Flash uniquement (pas de coût RAM)
const char monthNames[] PROGMEM = "JanFebMarAprMayJunJulAugSepOctNovDec";
char buildDateISO[20]; // "YYYY-MM-DD HH:MM:SS\0"
#endif

// Capteur d'orage (AS3935 en SPI)
AS3935 lightning;
const uint8_t LIGHTNING_INT = 0x08;
const uint8_t DISTURBER_INT = 0x04;
const uint8_t TOO_NOISY_INT = 0x01;
const uint8_t DISTANCE_MIN = 1;       // km (orage très proche)
const uint8_t DISTANCE_MAX = 40;      // km (limite portée AS3935)
const uint32_t ENERGIE_MIN = 0;
const uint32_t ENERGIE_MAX = 2097151; // Valeur max 21-bit retournée pas AS3935

bool isUptimeOvfl;
uint8_t uptimeWraps;  // Permet d'avoir 8 ans avant la saturation car millis() reboucle après 2^32 ms = 49,7 jours
uint32_t now;
uint32_t lastLedUpdate;
uint32_t buzzUntil;
bool buzzActive = false;

// This variable holds the number representing the lightning or non-lightning
// event issued by the lightning detector.
uint8_t  lightning_intReason =  UINT8_MAX;
uint8_t  lightning_distance  =  UINT8_MAX;
uint32_t lightning_energy    = UINT32_MAX;

byte energyLed = 0;

bool buzzerEnabled;
int buzzerFrequency;
int buzzerDuration;

// Statistiques
uint32_t disturberCount;
bool isDisturberCountOvfl;
uint32_t noiseCount;
bool isNoiseCountOvfl;
uint32_t lightningCount;
bool isLightningCountOvfl;
uint32_t lightningEnergyMinimum;
uint32_t lightningEnergyMaximum;
uint32_t lightningEnergyTotal;
bool isLightningEnergyOvfl;
uint32_t lightningDistanceMinimum;
uint32_t lightningDistanceMaximum;
uint32_t lightningDistanceTotal;
bool isLightningDistanceOvfl;
uint32_t distanceCount;
bool isDistanceCountOvfl;
uint32_t distanceMinimum;
uint32_t distanceMaximum;
uint32_t distanceTotal;
bool isDistanceOvfl;
uint32_t energyCount;
bool isEnergyCountOvfl;
uint32_t energyMinimum;
uint32_t energyMaximum;
uint32_t energyTotal;
bool isEnergyOvfl;
long voltVcc;
float temperature;

/**
 * @brief Démarre le buzzer passif (D3/OC2B obligatoirement) à la fréquence et pour la durée données.
 *
 * Remplace tone()/noTone() afin d'économiser de la mémoire Flash. L'extinction du son n'est pas
 * bloquante : elle est effectuée dans loop() une fois le délai `buzzUntil` atteint.
 * @param freq Fréquence du son en Hz.
 * @param ms Durée du son en millisecondes.
 */
static void buzz(uint16_t freq, uint16_t ms)
{
#ifdef BUZZER_OC2B
  if (freq < 31) { TCCR2A = TCCR2B = 0; return; }
  // Mode CTC (WGM22:0 = 010), bascule OC2B à chaque comparaison
  uint32_t top; uint8_t cs;
  for (cs = 1; cs <= 7; cs++)
  {
    static const uint16_t presc[] PROGMEM = {0, 1, 8, 32, 64, 128, 256, 1024};
    top = (F_CPU / (2UL * pgm_read_word(&presc[cs]) * freq)) - 1;
    if (top <= 255) break;
  }
  OCR2A = (uint8_t)top;                 // OCR2A définit la période en mode CTC
  OCR2B = (uint8_t)top;
  TCCR2A = _BV(COM2B0) | _BV(WGM21);    // bascule OC2B, CTC
  TCCR2B = cs;
#else
  tone(passiveBuzzer, freq);
#endif
  buzzUntil = millis() + ms;            // extinction non bloquante dans loop()
  buzzActive = true;
}

/**
 * @brief Émet un bref bip système si le buzzer est activé (utilisé notamment par pushError()).
 */
void systemBeep()
{
  if (buzzerEnabled) buzz(buzzerFrequency, buzzerDuration);
}

/**
 * @brief Réinitialise les réglages du buzzer et du capteur AS3935 à leurs valeurs par défaut (commande *RST).
 */
void resetSettings()
{ // *RST
  buzzerEnabled = buzzerStateDefault;
  buzzerFrequency = buzzerFreqDefault;
  buzzerDuration = (int)(buzzerDuraDefault * 1000.0);
  if (isAs3935Available) lightning.resetSettings();
}

/**
 * @brief Réinitialise un registre SCPI (CONDition, EVENt, PTRansition, NTRansition, MAP) à son état par défaut.
 * @param r Registre SCPI à réinitialiser.
 */
void regPreset(ScpiRegister &r)
{
  r.ena = ((r.ena & 0x8000) == 0)?0x0000:0xFFFF;
  r.ptr = 0x7FFF;
  r.ntr = 0x0000;
#ifdef SCPI_MAP
  if (r.map != nullptr)
  {
    for (uint8_t i = 0; i < 15; i++)
    { // Supprime le mapping
      r.map[i] = 0x0000;
    }
  }
#endif
}

/**
 * @brief Positionne un bit du registre CONDition et propage l'événement vers EVENt en cas de transition 0→1 autorisée par PTR.
 * @param r Registre SCPI concerné.
 * @param bit Indice du bit (0-14) à positionner.
 */
void regBitSet(ScpiRegister &r, uint8_t bit)
{
  if (bit < 15)
  {
    bool transition = !bitRead(r.con, bit); // 0 -> 1
    bitSet(r.con, bit);
    if (transition && bitRead(r.ptr, bit))
    {
      bitSet(r.eve, bit);
      updateStb();
    }
  }
}

/**
 * @brief Efface un bit du registre CONDition et propage l'événement vers EVENt en cas de transition 1→0 autorisée par NTR.
 * @param r Registre SCPI concerné.
 * @param bit Indice du bit (0-14) à effacer.
 */
void regBitClear(ScpiRegister &r, uint8_t bit)
{
  if (bit < 15)
  {
    bool transition = bitRead(r.con, bit); // 1 -> 0
    bitClear(r.con, bit);
    if (transition && bitRead(r.ntr, bit))
    {
      bitSet(r.eve, bit);
      updateStb();
    }
  }
}

/**
 * @brief Positionne ou efface un bit du registre CONDition selon la valeur donnée (appelle regBitSet() ou regBitClear()).
 * @param r Registre SCPI concerné.
 * @param bit Indice du bit (0-14) à écrire.
 * @param value État à écrire (true = positionner, false = effacer).
 */
void regBitWrite(ScpiRegister &r, uint8_t bit, bool value)
{
  if (value)
  {
    regBitSet(r, bit);
  }
  else
  {
    regBitClear(r, bit);
  }
}

/**
 * @brief Indique si au moins un bit actif du registre EVENt est autorisé par ENABle (cascade du registre).
 * @param r Registre SCPI à évaluer.
 * @return true si le registre doit remonter un état actif vers son parent.
 */
bool regCascade(ScpiRegister &r)
{
  return (r.eve & (r.ena & 0x7FFF)) != 0x0000;
}

/**
 * @brief Efface les registres EVENt de tous les registres SCPI ainsi que l'ESR (commande *CLS).
 */
void regClearEvents()
{
  operReg.eve = 0x0000;
  quesReg.eve = 0x0000;
  voltReg.eve = 0x0000;
  timeReg.eve = 0x0000;
  tempReg.eve = 0x0000;
  caliReg.eve = 0x0000;
  lighReg.eve = 0x0000;
  scpiEsr = 0x00;
  updateStb();
}

/**
 * @brief Réinitialise tous les registres SCPI de l'appareil à leur état par défaut.
 */
void regPresetAll()
{
  regPreset(operReg);
  regPreset(quesReg);
  regPreset(voltReg);
  regPreset(timeReg);
  regPreset(tempReg);
  regPreset(caliReg);
  regPreset(lighReg);
  updateStb();
}

/**
 * @brief Recalcule l'état cascadé du registre OPERation.
 * @return true si OPERation doit être signalé actif dans le Status Byte.
 */
bool updateOper()
{
  return regCascade(operReg);
}

/**
 * @brief Met à jour les bits de synthèse de QUEStionable (VOLTage, TIME, TEMPerature, CALibration, LIGHtning) puis recalcule son état cascadé.
 * @return true si QUEStionable doit être signalé actif dans le Status Byte.
 */
bool updateQues()
{
  regBitWrite(quesReg, QUES_VOLT, regCascade(voltReg));
  regBitWrite(quesReg, QUES_TIME, regCascade(timeReg));
  regBitWrite(quesReg, QUES_TEMP, regCascade(tempReg));
  regBitWrite(quesReg, QUES_CAL, regCascade(caliReg));
  regBitWrite(quesReg, QUES_LIGH, regCascade(lighReg));
  return regCascade(quesReg);
}

/**
 * @brief Recalcule entièrement le registre STB (Status Byte) à partir de la file d'erreurs, des registres
 * QUEStionable/OPERation, de l'état du buffer de sortie série et des masques ESE/SRE.
 */
void updateStb()
{
  // Les bit 0 et 1 sont spécifiques / non utilisés

  // Le bit 2 de STB (EAV) passe à 1 si la file d'attente des erreurs n'est pas vide
  bitWrite(scpiStb, 2, (queueCount > 0));
  digitalWrite(errorLed, queueCount > 0);

  // Le bit 3 de STB (QUES) passe à 1 si au moins un bit actif de QUES:EVEN est autorisé
  // dans QUES:ENAB. (Note: le bit 15 de QUES:* vaut toujours 0)
  bitWrite(scpiStb, 3, updateQues());

  // Le bit 4 de STB (MAV) passe à 1 si le buffer de sortie contient un message
  bitWrite(scpiStb, 4, Serial.availableForWrite() < (SERIAL_TX_BUFFER_SIZE - 1));

  // Le bit 5 de STB (ESB) passe à 1 si au moins un bit actif de ESR est autorisé dans ESE
  bitWrite(scpiStb, 5, scpiEsr & scpiEse);

  // Le bit 7 de STB (OPER) passe à 1 si au moins un bit actif de OPER:EVEN est autorisé
  // dans OPER:ENAB. (Note: le bit 15 de OPER:* vaut toujours 0)
  bitWrite(scpiStb, 7, updateOper());

  // Le bit 6 de STB (MSS) passe à 1 si au moins un bit actif de STB est autorisé dans SRE
  // (Note: le bit 6 de SRE est ignoré dans le masque)
  bitWrite(scpiStb, 6, scpiStb & (scpiSre & ~(1 << 6))); // Doit être calculé en dernier car dépend des autres
}

/**
 * @brief Analyse et traite les sous-commandes génériques CONDition/ENABle/EVENt/MAP/PTRansition/NTRansition
 * d'un registre SCPI (OPERation, QUEStionable, QUEStionable:LIGHtning, ...).
 * @param r Registre SCPI ciblé (r.map == nullptr désactive la sous-commande MAP pour ce registre).
 * @param subtoken Sous-commande courante (ex: "ENABle"), ou NULL si aucune (registre lui-même).
 * @param isQuery true si c'est une requête (suffixe '?').
 * @param argc Nombre d'arguments fournis.
 * @param argv Tableau des arguments.
 * @param rc [in,out] Code d'erreur SCPI, mis à jour en cas de problème (0 = ok). Rien n'est fait si déjà non nul en entrée.
 */
void parseScpiRegister(ScpiRegister &r, char *subtoken, bool isQuery, int argc, char *argv[], int16_t &rc)
{
  if (isQuery && isToken(subtoken, F("CONDition")))
  { // ...:CONDition?
    rc = compareArgumentsCount(0, argc);
    if (rc == 0)
    {
      displaySeparator();
      displayInteger(r.con);
    }
  }
  else if (isToken(subtoken, F("ENABle")))
  { // ...:ENABle? & ...:ENABle <NR1>
    int32_t v = r.ena & 0x7FFF;
    bool isDefOnes = (r.ena & 0x8000) != 0;
    if (checkInteger(isQuery, argc, argv, 0, isDefOnes?0x7FFF:0x0000, UINT16_MAX, v, rc))
    {
      r.ena = (v & 0x7FFF) | isDefOnes?0x8000:0x0000;
      updateStb();
    }
  }
  else if (isQuery && ((subtoken == NULL) || isToken(subtoken, F("EVENt"))))
  { // ...[:EVENt]?
    rc = compareArgumentsCount(0, argc);
    if (rc == 0)
    {
      displaySeparator();
      displayInteger(r.eve);
      r.eve = 0;
    }
  }
#ifdef SCPI_MAP
  else if ((r.map != nullptr) && isToken(subtoken, F("MAP")))
  { // ...:MAP? [<NR1>] & ...:MAP <NR1>,<NR1>
    int32_t bit = -1;
    int32_t event;
    if (isQuery)
    {
      bool comma = false;
      switch (argc)
      {
        case 0:
          displaySeparator();
          for (int i = 0; i < 15; i++)
          {
            if (r.map[i] != 0)
            {
              if (comma) Serial.write(',');
              displayInteger(i);
              Serial.write(',');
              displayInteger(r.map[i]);
              comma = true;
            }
          }
          scpiOutput = true;
          break;
        case 1:
          rc = isNR1(argv[0], bit);
          if (rc == 0)
          {
            if ((bit < 0) || (bit > 14)) // bit 15 banni
            {
              rc = -222; // Data out of range
            }
            else
            {
              displaySeparator();
              displayInteger(r.map[bit]);
            }
          }
          break;
        default:
          rc = -108;
          break;
      }
    }
    else
    {
      rc = compareArgumentsCount(2, argc);
      if (rc == 0)
      {
        rc = isNR1(argv[0], bit);
        if (rc == 0)
        {
          rc = isNR1(argv[1], event);
        }
        if (rc == 0)
        {
          if ((bit < 9) || (bit > 12)) // Plage commune à tous les registres avec MAP
          {
            rc = -222; // Data out of range
          }
          else
          {
            r.map[bit] = event;
          }
        }
      }
    }
  }
#endif
  else if (isToken(subtoken, F("NTRansition")))
  { // ...:NTRansition? & ...:NTRansition <NR1>
    int32_t v = r.ntr;
    if (checkInteger(isQuery, argc, argv, 0, 0, UINT16_MAX, v, rc))
    {
      r.ntr = v & 0x7FFF;
      updateStb();
    }
  }
  else if (isToken(subtoken, F("PTRansition")))
  { // ...:PTRansition? & ...:PTRansition <NR1>
    int32_t v = r.ptr;
    if (checkInteger(isQuery, argc, argv, 0, 0x00007FFF, UINT16_MAX, v, rc))
    {
      r.ptr = v & 0x7FFF;
      updateStb();
    }
  }
  else
  {
    rc = -113;
  }
}

/**
 * @brief Réinitialise les statistiques sélectionnées (foudre, énergie, distance, parasites, bruit).
 * @param flag Combinaison de StatFlag indiquant les catégories de statistiques à réinitialiser.
 */
void resetStatistics(StatFlag flag)
{
  if (flag & STAT_LIGHTNING)
  {
    lightningCount = 0;
    isLightningCountOvfl = false;
    lightningEnergyMinimum = UINT32_MAX;
    lightningEnergyMaximum = 0;
    lightningEnergyTotal = 0;
    isLightningEnergyOvfl = false;
    lightningDistanceMinimum = UINT32_MAX;
    lightningDistanceMaximum = 0;
    lightningDistanceTotal = 0;
    isLightningDistanceOvfl = false;
  }
  if (flag & STAT_ENERGY)
  {
    energyCount = 0;
    isEnergyCountOvfl = false;
    energyMinimum = UINT32_MAX;
    energyMaximum = 0;
    energyTotal = 0;
    isEnergyOvfl = false;
  }
  if (flag & STAT_DISTANCE)
  {
    distanceCount = 0;
    isDistanceCountOvfl = false;
    distanceMinimum = UINT32_MAX;
    distanceMaximum = 0;
    distanceTotal = 0;
    isDistanceOvfl = false;
  }
  if (flag & STAT_DISTURBER)
  {
    disturberCount = 0;
    isDisturberCountOvfl = false;
  }
  if (flag & STAT_NOISE)
  {
    noiseCount = 0;
    isNoiseCountOvfl = false;
  }
}

#ifndef FIRMWARE_VERSION
/**
 * @brief Construit la date de compilation au format ISO ("YYYY-MM-DD HH:MM:SS") à partir des macros
 * __DATE__ et __TIME__, et la stocke dans buildDateISO.
 */
void formatBuildDate()
{
  const char *date = __DATE__; // "Aug  2 2026"
  const char *time = __TIME__; // "14:32:10"

  // Trouver le mois (1-12) en cherchant les 3 premières lettres dans la table
  char monthStr[4] = { date[0], date[1], date[2], '\0' };
  uint8_t month = 0;
  for (uint8_t i = 0; i < 12; i++)
  {
    char ref[4];
    memcpy_P(ref, monthNames + (i * 3), 3);
    ref[3] = '\0';
    if (strcmp(monthStr, ref) == 0)
    {
      month = i + 1;
      break;
    }
  }

  // Jour : gérer l'espace à la place du zéro (ex: " 2" -> "02")
  // Année : caractères 7 à 10
  buildDateISO[0] = date[7];
  buildDateISO[1] = date[8];
  buildDateISO[2] = date[9];
  buildDateISO[3] = date[10];
  buildDateISO[4] = '-';
  buildDateISO[5] = '0' + (month / 10);
  buildDateISO[6] = '0' + (month % 10);
  buildDateISO[7] = '-';
  buildDateISO[8] = (date[4] == ' ')?'0':date[4];
  buildDateISO[9] = date[5];
  buildDateISO[10] = ' ';
  for (uint8_t i = 0; i < 8; i++)
  {
    buildDateISO[11 + i] = time[i];
  }
  buildDateISO[19] = '\0';
}
#endif

/**
 * @brief Calcule un CRC8 (polynôme 0x31) sur un bloc de données, en ignorant l'octet d'indice 1 (réservé au champ checksum).
 * @param p Pointeur vers les données.
 * @param n Taille des données en octets.
 * @return Valeur du CRC8 calculé.
 */
uint8_t crc8(const uint8_t *p, size_t n)
{
  uint8_t crc = 0xFF;     // valeur initiale non nulle
  for (size_t i = 0; i < n; i++)
  {
    if (i == 1) continue; // CRC est à l'octet p[1]
    crc ^= p[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

#ifdef SCPI_PUD
// Données personnelles utilisateur
#define PUD_BASE 0x040
#define PUD_SIZE 60
struct SavedPud
{ // 62 bytes
  uint8_t len;          // Longueur des données *PUD
  uint8_t checksum;     // CRC8
  char data[PUD_SIZE];  // Ne termine pas forcément par '\0'
};

/**
 * @brief Enregistre en EEPROM les données utilisateur protégées (*PUD), après validation du format bloc IEEE 488.2.
 * @param s Chaîne au format bloc définitif ("#<nb_chiffres><longueur><données>").
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t saveProtectedUsedData(const char *s)
{ // *PUD <data>
  int16_t rc;
  uint8_t lens = strlen(s);
  if ((lens < 3) || (s[0] != '#') || (s[1] < '1') || (s[1] > '9'))
  {
    rc = -161; // Invalid block data
  }
  uint8_t len = 0;
  lens -= 2;
  for (uint8_t i = 0; i < (s[1] - '0'); i++)
  {
    len *= 10;
    len += s[i + 2] - '0';
    lens--;
  }
  if (isProtected)
  {
    rc = -203; // Command protected
  }
  else if ((len > PUD_SIZE) || (lens > len))
  {
    rc = -223; // Too much data
  }
  else if (len == lens)
  {
    rc = 0;
    SavedPud pud;
    pud.len = len;
    memcpy(pud.data, &s[s[1] - '0' + 2], len); // len <= PUD_SIZE
    uint8_t crc = crc8(reinterpret_cast<const uint8_t*>(&pud), sizeof(SavedPud));
    pud.checksum = crc;
    EEPROM.put(PUD_BASE, pud);

    //Vérification
    EEPROM.get(PUD_BASE, pud);
    if (crc8(reinterpret_cast<const uint8_t*>(&pud), sizeof(SavedPud)) != crc)
    {
      rc = -311; // Memory error
    }
  }
  else
  {
    rc = -161; // Invalid block data
  }
  return rc;
}

/**
 * @brief Charge depuis l'EEPROM les données utilisateur protégées et vérifie leur intégrité par CRC.
 * @param pud [out] Structure recevant les données lues.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t loadProtectedUsedData(struct SavedPud &pud)
{ // *PUD?
  int16_t rc;
  EEPROM.get(PUD_BASE, pud);
  uint8_t crc = crc8(reinterpret_cast<const uint8_t*>(&pud), sizeof(SavedPud));
  if (crc != pud.checksum)
  {
    rc = -312; // PUD memory lost
  }
  else
  {
    rc = (pud.len > PUD_SIZE)?-223:0; // Too much data
  }
  return rc;
}
#endif

/**
 * @brief Sauvegarde la configuration courante du capteur AS3935 et du buzzer dans un emplacement EEPROM (*SAV).
 * @param slot Numéro d'emplacement (0 à SLOT_COUNT-1).
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t saveState(uint8_t slot)
{ // *SAV 0-(SLOT_COUNT - 1)
  SavedState s;
  int16_t rc = 0;
  if (slot > (SLOT_COUNT - 1))
  {
    rc = -310; // System error
  }
  else if (!isAs3935Available)
  {
    rc = -241; // Hardware missing
  }
  else
  {
    size_t addr = SLOT_BASE + (slot * SLOT_SIZE);
    s.magic = SLOT_MAGIC;
    s.version = SLOT_VERSION;
    // AS3935
    s.afeMd = (lightning.readIndoorOutdoor() & 0x7f) | (lightning.readMaskDisturber()?0x80:0x00);
    s.noiseFloor = lightning.readNoiseLevel();
    s.watchDogVal = lightning.readWatchdogThreshold();
    s.spike = lightning.readSpikeRejection();
    s.lightningThresh = lightning.readLightningThreshold();
    // Conf interne
    s.buzzerFreq = buzzerFrequency;
    s.buzzerDura = (buzzerDuration & 0x7fff) | (buzzerEnabled?0x8000:0x0000); // BIT 15 = buzzerEnabled

    // Calcul CRC, écriture et vérification
    uint8_t checksum = crc8(reinterpret_cast<const uint8_t*>(&s), sizeof(SavedState));
    s.checksum = checksum;
    EEPROM.put(addr, s);
    
    //Vérification
    EEPROM.get(addr, s);
    if (crc8(reinterpret_cast<const uint8_t*>(&s), sizeof(SavedState)) != checksum)
    {
      rc = -311; // Memory error
    }
  }
  return rc;
}

/**
 * @brief Restaure la configuration du capteur AS3935 et du buzzer depuis un emplacement EEPROM (*RCL).
 * @param slot Numéro d'emplacement (0 à SLOT_COUNT-1).
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t recallState(uint8_t slot)
{ // *RCL 0-(SLOT_COUNT-1)
  SavedState s;
  int16_t rc = 0;
  if (slot > (SLOT_COUNT -1))
  {
    rc = -310; // System error
  }
  else if (!isAs3935Available)
  {
    rc = -241; // Hardware missing
  }
  else
  {
    size_t addr = SLOT_BASE + (slot * SLOT_SIZE);
    EEPROM.get(addr, s);
    uint8_t checksum = crc8(reinterpret_cast<const uint8_t*>(&s), sizeof(SavedState));
    if ((checksum != s.checksum) || (s.magic != SLOT_MAGIC))
    {
      rc = -315; // Configuration memory lost
    }
    else if (s.version != SLOT_VERSION)
    {
      rc = -233; // Invalid version
    }
    else
    {
      // AS3935
      lightning.setIndoorOutdoor(s.afeMd & 0x7f);
      isDisturberMasked = bitRead(s.afeMd, 7);
      lightning.maskDisturber(isDisturberMasked);
      lightning.setNoiseLevel(s.noiseFloor);
      lightning.watchdogThreshold(s.watchDogVal);
      lightning.spikeRejection(s.spike);
      lightning.lightningThreshold(s.lightningThresh);
      // Conf interne
      buzzerFrequency = s.buzzerFreq;
      buzzerDuration = s.buzzerDura & 0x7FFF;
      buzzerEnabled = bitRead(s.buzzerDura, 15); // BIT 15 = buzzerEnabled
    }
  }
  return rc;
}

#ifdef SCPI_PSC
/**
 * @brief Sauvegarde en EEPROM les masques ESE/SRE et les registres d'activation (ENABle) pour restauration au prochain démarrage.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t savePowerOnStatus()
{ // *PSC = Power on Status Clear
  SavedPonStatus pos;
  int16_t rc = 0;
  pos.version = CONF_VERSION | (isPSC?0x80:0x00);
  pos.ese = scpiEse;
  pos.sre = scpiSre;
  pos.oper = operReg.ena;
  pos.ques = quesReg.ena;
  pos.volt = voltReg.ena;
  pos.time = timeReg.ena;
  pos.temp = tempReg.ena;
  pos.cali = caliReg.ena;
  pos.ligh = lighReg.ena;
  uint8_t checksum = crc8(reinterpret_cast<const uint8_t*>(&pos), sizeof(SavedPonStatus));
  pos.checksum = checksum;
  EEPROM.put(CONF_BASE, pos);

  //Vérification
  EEPROM.get(CONF_BASE, pos);
  if (crc8(reinterpret_cast<const uint8_t*>(&pos), sizeof(SavedPonStatus)) != checksum)
  {
    rc = -311; // Memory error
  }
  return rc;
}

/**
 * @brief Recharge depuis l'EEPROM les masques ESE/SRE et les registres ENABle, si *PSC est à OFF.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t loadPowerOnConfiguration()
{
  SavedPonStatus pos;
  int16_t rc = 0;
  EEPROM.get(CONF_BASE, pos);
  uint8_t checksum = crc8(reinterpret_cast<const uint8_t*>(&pos), sizeof(SavedPonStatus));
  if (checksum != pos.checksum)
  {
    rc = -315; // Configuration memory lost
  }
  else if ((pos.version & 0x7F) != CONF_VERSION)
  {
    rc = -233; // Invalid version
  }
  else
  {
    isPSC = pos.version & 0x80;
    if (!isPSC)
    {
      scpiEse = pos.ese;
      scpiSre = pos.sre;
      operReg.ena = pos.oper;
      quesReg.ena = pos.ques;
      voltReg.ena = pos.volt;
      timeReg.ena = pos.time;
      tempReg.ena = pos.temp;
      caliReg.ena = pos.cali;
      lighReg.ena = pos.ligh;
      updateStb();
    }
  }
  return rc;
}
#endif

// Données de calibration (chargées à l'allumage)
// Non modifiée par *CAL? mais uniquement par :CALibration:STORe (après déverrouillage)
#define CALI_BASE 0x100
#define CALI_VERSION 1
#define CALI_MAGIC 0x5A
struct SavedCalibration
{ // 86 bytes
  uint8_t version;      // Version 1
  uint8_t checksum;     // CRC8
  uint16_t count;       // Nombre de modification des données dans cette structure
  char passwd[12];      // Ne termine pas forcément par '\0'. Est sauvegardé dès modification.
  uint8_t flags;
  uint8_t capafreq[3];  // 4 bits de poids fort = capacité. 20 bits de poids faible = fréquence mesurée
  uint16_t caldate;     // Année (7 bits) + mois (4 bits) + jour (5 bits)
  uint16_t duedate;     // Date fin de validité de la calibration
  float caltemp;        // Température lors de la calibration
  char note[40];        // Ne termine pas forcément par '\0'
  float t_offset;       // 
  float t_gain;
} cal;
char calString[sizeof(cal.note)];
uint16_t calibrationDate, calibrationDueDate;

enum : uint8_t
{
  CAL_SOUR_NONE = 0,
  CAL_SOUR_TRCO = 1,
  CAL_SOUR_SRCO = 2,
  CAL_SOUR_LCO = 3
};
uint8_t sourceCal = CAL_SOUR_NONE;

/**
 * @brief Met à jour et sauvegarde en EEPROM les données de calibration (capacité d'accord, date, température), puis les recharge.
 * @param passwdOnly Si true, seul le mot de passe est considéré modifié ; les autres données de calibration ne sont pas recalculées.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI (voir loadCalibration()).
 */
int16_t saveCalibration(bool passwdOnly)
{
  cal.count++;
  if (!passwdOnly)
  {
    // bitWrite(cal.flags, 0, isCalibrated);
    if (calibrationDate == 0)
    {
      calibrationDate = currentDateToWord();
    }
    uint8_t cap = lightning.readTuneCap();
    cap <<= 4;
    cal.capafreq[0] &= 0x0F;
    cal.capafreq[0] |= cap;
    cal.caldate = calibrationDate;
    cal.duedate = calibrationDueDate>calibrationDate?calibrationDueDate:0;
    readTemperature();
    cal.caltemp = temperature;
    memcpy(cal.note, calString, sizeof(cal.note));
  }
  cal.checksum = crc8(reinterpret_cast<const uint8_t*>(&cal), sizeof(SavedCalibration));
  EEPROM.put(CALI_BASE, cal);
  return loadCalibration();
}

/**
 * @brief Charge et vérifie les données de calibration depuis l'EEPROM ; applique des valeurs par défaut
 * "non calibré" en cas d'échec et met à jour les bits d'état QUEStionable:CALibration.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t loadCalibration()
{
  int16_t rc = 0;
  EEPROM.get(CALI_BASE, cal);
  uint8_t checksum = crc8(reinterpret_cast<const uint8_t*>(&cal), sizeof(SavedCalibration));
  if (checksum != cal.checksum)
  {
    rc = -313; // Calibration memory lost
  }
  else if (cal.version != CALI_VERSION)
  {
    rc = -233; // Invalid version
  }
  if (rc != 0)
  { // Load default (un)calibrated parameters
    cal.version = CALI_VERSION;
    cal.count = 0;
    strncpy(cal.passwd, "VTX1234", sizeof(cal.passwd) - 1);
    cal.caldate = 0;
    cal.duedate = 0;
    cal.caltemp = SCPI_NAN;
    strncpy(cal.note, "NOT CALIBRATED!", sizeof(cal.note) - 1);
    cal.t_offset = 324.31;  // valeur typique datasheet
    cal.t_gain = 1.22;      // valeur typique datasheet
  }
  calibrationDate = cal.caldate;
  calibrationDueDate = cal.duedate;
  for (size_t i = 0; i < sizeof(calString); i++)
  {
    calString[i] = cal.note[i];
  }
  regBitWrite(caliReg, CALI_CDCO, (rc != 0));
  uint8_t trco = (lightning.readRegister(0x3A) & 0xC0) ^ 0x80;
  uint8_t srco = (lightning.readRegister(0x3B) & 0xC0) ^ 0x80;
  regBitWrite(caliReg, CALI_NOTR, bitRead(trco, 7));
  regBitWrite(caliReg, CALI_TRCO, bitRead(trco, 6));
  regBitWrite(caliReg, CALI_NOSR, bitRead(srco, 7));
  regBitWrite(caliReg, CALI_SRCO, bitRead(srco, 6));
  return rc;
}

/**
 * @brief Écrit une valeur booléenne sur le port série au format SCPI ('0' ou '1').
 * @param value Valeur à afficher.
 */
void displayBoolean(bool value)
{
  Serial.write(value?'1':'0');
  scpiOutput = true;
}

/**
 * @brief Écrit un entier sur le port série au format SCPI NR1 (avec signe explicite).
 * @param value Valeur à afficher.
 */
void displayInteger(int32_t value)
{
  if (value >= 0) Serial.write('+');
  Serial.print(value);
  scpiOutput = true;
}

/**
 * @brief Écrit un nombre flottant sur le port série au format SCPI NR3 (mantisse + exposant, ex: +1.23456E+02).
 * @param value Valeur à afficher.
 */
void displayFloat(float value)
{
  if (value == 0.0)
  {
    Serial.print("+0.00000E+00");
  }
  else
  {
    int exponent = 0;
    float mantissa = value;
    
    while (mantissa < 1 && mantissa > -1) { mantissa *= 10; exponent--; }
    while (mantissa >= 10 || mantissa <= -10) { mantissa /= 10; exponent++; }
    
    if (mantissa > 0) Serial.write('+');
    Serial.print(mantissa, 5);  // 5 décimales
    Serial.write('E');
    Serial.write((exponent >= 0)?'+':'-');
    if ((exponent > -10) && (exponent < 10)) Serial.write('0');
    Serial.print(abs(exponent));
  }
  scpiOutput = true;
}

/**
 * @brief Décode et affiche une date compactée (format interne 16 bits) au format SCPI "année,mois,jour".
 * @param d Date compactée à décoder et afficher.
 */
void displayDate(uint16_t d)
{
  int a, m, j;
  a = (d & 0xFE00) >> 9;
  a += 2000;
  m = (d & 0x01E0) >> 5;
  j = d & 0x001F;
  if ((m < 1) || (m > 12) || (d == 0))
  {
    a = 0;
    m = 0;
    j = 0;
  }
  displayInteger(a);
  Serial.write(',');
  displayInteger(m);
  Serial.write(',');
  displayInteger(j);
  scpiOutput = true;
}

/**
 * @brief Décode et affiche une heure compactée (format interne 16 bits) au format SCPI "heure,minute,seconde".
 * @param d Heure compactée à décoder et afficher.
 */
void displayTime(uint16_t d)
{
  int h, m, s;
  h = (d & 0xF100) >> 11;
  m = (d & 0x07E0) >> 5;
  s = (d & 0x001F) << 1;
  if ((h > 23) || (m > 59) || (s > 59))
  {
    h = 0;
    m = 0;
    s = 0;
  }
  displayInteger(h);
  Serial.write(',');
  displayInteger(m);
  Serial.write(',');
  displayInteger(s);
  scpiOutput = true;
}

/**
 * @brief Lit le numéro de série stocké en EEPROM (avec vérification par complément à 1) et l'affiche en hexadécimal.
 */
void displaySerial()
{
  for (size_t i = 0; i < 8; i++)
  {
    uint8_t sn = EEPROM.read(i);
    if (EEPROM.read(i+8) == ~sn)
    {
      for (uint8_t j = 0; j < 2; j++)
      {
        uint8_t nibble = (sn & 0xF0) >> 4;
        nibble += (nibble > 9)?55:'0'; // 55 = 'A' - 10 
        Serial.write(nibble);
        sn = sn << 4;
      }
    }
    else
    {
      Serial.print(F("xx"));
    }
  }
  scpiOutput = true;
}

/**
 * @brief Affiche une chaîne entre guillemets au format SCPI, en doublant les guillemets internes.
 * @param s Chaîne à afficher (peut ne pas être terminée par '\0').
 * @param l Longueur maximale à afficher.
 */
void displayString(const char* s, int l)
{
  Serial.write('"');
  size_t i = 0;
  while ((i < l) && (s[i] != '\0'))
  {
    if (s[i] == '"') Serial.write('"');
    Serial.write(s[i++]);
  }
  Serial.write('"');
  scpiOutput = true;
}

/**
 * @brief Affiche des données binaires au format bloc définitif IEEE 488.2 ("#<n><longueur><données>").
 * @param data Pointeur vers les données binaires.
 * @param len Longueur des données en octets.
 */
void displayBlock(const uint8_t* data, uint16_t len)
{
  char header[7];
  uint8_t headerLen = 0;
  uint16_t i = len;
  do
  {
    header[sizeof(header) - (++headerLen)] = '0' + (i % 10);
    i /= 10;
  } while (i > 9);
  header[sizeof(header) - (++headerLen)] = '0' - 1 + headerLen;
  header[sizeof(header) - (++headerLen)] = '#';
  Serial.write(&header[sizeof(header) - headerLen], headerLen);
  Serial.write(data, len);
  scpiOutput = true;
}

/**
 * @brief Écrit un séparateur ';' avant une nouvelle valeur de réponse, si une valeur a déjà été affichée.
 */
void displaySeparator()
{
  if (scpiOutput) Serial.write(';');
}

/**
 * @brief Affiche la valeur spéciale "Not a Number" (SCPI_NAN) au format flottant.
 */
void displayNaN()
{
  displayFloat(SCPI_NAN);
}

/**
 * @brief Affiche la valeur spéciale infini positif (SCPI_PINFINITY) au format flottant.
 */
void displayPInfinity()
{
  displayFloat(SCPI_PINFINITY);
}

/**
 * @brief Affiche la valeur spéciale infini négatif (SCPI_NINFINITY) au format flottant.
 */
void displayNInfinity()
{
  displayFloat(SCPI_NINFINITY);
}

/**
 * @brief Affiche un code d'erreur SCPI suivi de son libellé textuel : "<code>,\"<message>\"" (réponse à SYSTem:ERRor?).
 * @param error Code d'erreur SCPI à afficher.
 */
void displayError(int16_t error)
{
  displayInteger(error);
  Serial.print(",\"");
  switch (error)
  { // §21.16 1999 SCPI Command Reference
    case 0:
      Serial.print(F("No error"));
      break;
    // case -100:
    //   Serial.print(F("Command error"));
    //   break;
    case -101:
      Serial.print(F("Invalid character"));
      break;
    case -102:
      Serial.print(F("Syntax error"));
      break;
    case -103:
      Serial.print(F("Invalid separator"));
      break;
    case -108:
      Serial.print(F("Parameter not allowed"));
      break;
    case -109:
      Serial.print(F("Missing parameter"));
      break;
    case -110:
      Serial.print(F("Command header error"));
      break;
    case -112:
      Serial.print(F("Program mnemonic too long"));
      break;
    case -113:
      Serial.print(F("Undefined header"));
      break;
    case -120:
      Serial.print(F("Numeric data error"));
      break;
    case -121:
      Serial.print(F("Invalid character in number"));
      break;
    case -123:
      Serial.print(F("Exponent too large"));
      break;
    case -124:
      Serial.print(F("Too many digits"));
      break;
    // case -200:
    //   Serial.print(F("Execution error"));
    //   break;
    case -151:
      Serial.print(F("Invalid string data"));
      break;
    case -161:
      Serial.print(F("Invalid block data"));
      break;
    case -203:
      Serial.print(F("Command protected"));
      break;
    // case -220:
    //   Serial.print(F("Parameter error"));
    //   break;
    case -221:
      Serial.print(F("Settings conflict"));
      break;
    case -222:
      Serial.print(F("Data out of range"));
      break;
    case -223:
      Serial.print(F("Too much data"));
      break;
    case -233:
      Serial.print(F("Invalid version"));
      break;
    case -224:
      Serial.print(F("Illegal parameter value"));
      break;
    case -240:
      Serial.print(F("Hardware error"));
      break;
    case -241:
      Serial.print(F("Hardware missing"));
      break;
    case -310:
      Serial.print(F("System error"));
      break;
    case -311:
      Serial.print(F("Memory error"));
      break;
#ifdef SCPI_PUD
    case -312:
      Serial.print(F("PUD memory lost"));
      break;
#endif
    case -313:
      Serial.print(F("Calibration memory lost"));
      break;
    case -314:
      Serial.print(F("Save/recall memory lost"));
      break;
    case -315:
      Serial.print(F("Configuration memory lost"));
      break;
    case -330:
      Serial.print(F("Self-test failed"));
      break;
    case -340:
      Serial.print(F("Calibration failed"));
      break;
    case -350:
      Serial.print(F("Queue overflow"));
      break;
    case -500:
      Serial.print(F("Power on"));
      break;
  }
  Serial.write('"');
  scpiOutput = true;
}

/**
 * @brief Effectue l'auto-calibration des oscillateurs internes du capteur AS3935 et met à jour les bits d'état de calibration (*CAL?).
 * @param calibrationResult [out] Octet de résultat détaillant l'état des oscillateurs TRCO/SRCO.
 * @return 0 en cas de succès, sinon -340 (Calibration failed).
 */
uint16_t selfCalibration(uint8_t &calibrationResult)
{ // *CAL?
  calibrationResult = 0x00;
  // Measure and adjust internal settings :
  //  - bias voltage
  //  - gain
  //  - DAC/ADC accuracy
  //  - compensation
  // Update calibration value in non volatile memory

  if (!lightning.calibrateOsc())
  {
    calibrationResult = 0x01;
  }
  uint8_t trco = (lightning.readRegister(0x3A) & 0xC0) ^ 0x80;
  uint8_t srco = (lightning.readRegister(0x3B) & 0xC0) ^ 0x80;
  regBitWrite(caliReg, CALI_NOTR, bitRead(trco, 7));
  regBitWrite(caliReg, CALI_TRCO, bitRead(trco, 6));
  regBitWrite(caliReg, CALI_NOSR, bitRead(srco, 7));
  regBitWrite(caliReg, CALI_SRCO, bitRead(srco, 6));
  calibrationResult |= trco;
  calibrationResult |= srco>>2;
  // bit 3 (Device Error) of ESR should be set with this error code
  return  (calibrationResult == 0)?0:-340; // Calibration failed
}

/**
 * @brief Indique si la restauration de la calibration a échoué au démarrage (utilisé par le POST).
 * @return true en cas d'échec (implémentation actuelle : toujours false).
 */
bool isRestoreCalibrationFail()
{
  size_t addr = CALI_BASE;

  return false;
}

/**
 * @brief Exécute les vérifications du POST (Power On Self Test) et empile les erreurs correspondantes.
 * @return true si au moins un test du POST a échoué.
 */
bool isPostFail()
{
  bool fail = false;
  if (isRestoreCalibrationFail())
  {
    fail = true;
    pushError(-313);
  }
  return fail;
}

/**
 * @brief Exécute l'auto-test de l'appareil (*TST?) et construit bit à bit l'octet de résultat selon les sous-systèmes testés.
 * @param testResult [out] Octet de résultat détaillant les échecs par sous-système.
 * @return 0 en cas de succès, sinon -330 (Self-test failed).
 */
int16_t autoTest(uint8_t &testResult)
{ // *TST?
  testResult = 0x00;
  // Check basic hardware:
  //  - #7 POST (Power On Self Test)
  //  - #6 Test RAM/ROM/Flash
  //  - #5 Check bus (I2C, SPI, etc.)
  // Check measure and conversion subsystems:
  //  - #4 ADC/DAC
  //  - #3 references
  //  - #2 power rails
  // Check devices:
  //  - #1 AS3935
  //  - #0 Not used
  bitWrite(testResult, 7, isPostFail());
  //bitWrite(testResult, 2, isVccOutOfRange());
  bitWrite(testResult, 1, !isAs3935Available);

  // bit 3 (Device Error) of ESR should be set with this error code
  return (testResult == 0)?0:-330; // Self-test failed
}

/**
 * @brief Retourne le prochain jeton (segment d'en-tête SCPI) en poursuivant l'analyse strtok() sur les ':'.
 * @return Pointeur vers le prochain jeton, ou NULL s'il n'y en a plus.
 */
char *nextToken()
{
  return strtok(NULL, ":");
}

/**
 * @brief Compare une chaîne reçue à un mnémonique SCPI de référence stocké en Flash, en acceptant sa forme
 * courte (majuscules du mnémonique) ou sa forme longue complète.
 * @param s Chaîne à comparer (mnémonique reçu), insensible à la casse.
 * @param tokenF Mnémonique de référence complet, stocké en Flash (ex: F("LIGHtning")).
 * @return true si `s` correspond à la forme courte ou longue de `tokenF`.
 */
bool isToken(const char *s, const __FlashStringHelper *tokenF)
{
  if (s == NULL)
  {
    return false;
  }
  PGM_P token = reinterpret_cast<PGM_P>(tokenF);
  size_t tlen = strlen_P(token);
  size_t slen = strlen(s);
  bool isThisOne = true;
  if (slen == tlen)
  {
    for (size_t i = 0; isThisOne && (i < tlen); i++)
    {
      char ct = toupper(pgm_read_byte(token + i));
      char cs = toupper(s[i]);
      isThisOne &= (cs == ct);
    }
  }
  else if (slen < tlen)
  {
    size_t j = 0;
    size_t ss = 0;
    isThisOne = (slen > 0);
    for (size_t i = 0; isThisOne && (i < tlen); i++)
    {
      char ct = pgm_read_byte(token + i);
      if ((ct < 'a') || (ct > 'z'))
      {
        ss++;
        // ct = toupper(ct); // Inutile car ct ne peut pas être minuscule
        char cs = toupper(s[j++]);
        isThisOne &= ((cs == ct) && (j <= slen));
      }
    }
    isThisOne &= (ss == slen);
  }
  else
  {
    isThisOne = false;
  }
  return isThisOne;
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI DEFault.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à DEFault.
 */
bool isDefault(const char *s)
{
  return isToken(s, F("DEFault"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI MINimum.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à MINimum.
 */
bool isMinimum(const char *s)
{
  return isToken(s, F("MINimum"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI AVERage.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à AVERage.
 */
bool isAverage(const char *s)
{
  return isToken(s, F("AVERage"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI MAXimum.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à MAXimum.
 */
bool isMaximum(const char *s)
{
  return isToken(s, F("MAXimum"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI UP.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à UP.
 */
bool isUp(const char *s)
{
  return isToken(s, F("UP"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI DOWN.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à DOWN.
 */
bool isDown(const char *s)
{
  return isToken(s, F("DOWN"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI INFinity.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à INFinity.
 */
bool isInfinity(const char *s)
{
  return isToken(s, F("INFinity"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI NINFinity.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à NINFinity.
 */
bool isNInfinity(const char *s)
{
  return isToken(s, F("NINFinity"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI NAN.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à NAN.
 */
bool isNan(const char *s)
{
  return isToken(s, F("NAN"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI CLEar.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à CLEar.
 */
bool isClear(const char *s)
{
  return isToken(s, F("CLEar"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI COUNt.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à COUNt.
 */
bool isCount(const char *s)
{
  return isToken(s, F("COUNt"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI STATe.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à STATe.
 */
bool isState(const char *s)
{
  return isToken(s, F("STATe"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI THReshold.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à THReshold.
 */
bool isThreshold(const char *s)
{
  return isToken(s, F("THReshold"));
}

/**
 * @brief Indique si la chaîne correspond au mot-clé SCPI PERCent.
 * @param s Chaîne à tester.
 * @return true si `s` correspond à PERCent.
 */
bool isPercent(const char *s)
{
  return isToken(s, F("PERCent"));
}

/**
 * @brief Interprète une chaîne comme une valeur booléenne SCPI (ON/OFF, ou 0/1 avec zéros de poids fort tolérés).
 * @param s Chaîne à interpréter.
 * @param value [out] Valeur booléenne obtenue.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t isBool(const char *s, bool &value)
{
  int16_t rc = -310; // System error
  if (s == NULL) return -310; // System error
  if (isToken(s, F("OFF")))
  {
    value = false;
    return 0;
  }
  if (isToken(s, F("ON")))
  {
    value = true;
    return 0;
  }
  bool isValid = true;
  size_t slen = strlen(s);
  if (slen == 0) return -120; // Numeric data error
  slen--;
  for (size_t i = 0; i < slen; i++)
  {
    isValid &= (s[i] == '0');
  }
  if (isValid)
  {
    switch (s[slen])
    {
      case '0':
        value = false;
        break;
      case '1':
        value = true;
        break;
      default:
        isValid = false;
        break;
    }
  }
  return isValid?0:-120;
}

/**
 * @brief Interprète une chaîne comme un entier au format NR1 (décimal, ou octal #Q / hexadécimal #H / binaire #B, avec signe optionnel).
 * @param s Chaîne à interpréter.
 * @param value [out] Valeur entière obtenue.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t isNR1(const char *s, int32_t &value)
{
  int16_t rc = -310;
  uint8_t base = 10;
  size_t i = 0;
  if (s != NULL)
  {
    rc = 0;
    int32_t n = 0;
    bool negvalue = false;
    switch (s[i])
    {
      case '+':
        i++;
        break;
      case '-':
        i++;
        negvalue = true;
        break;
      case '#':
        i++;
        switch (s[i])
        {
          case 'q':
          case 'Q': // octal
            base = 8;
            i++;
            break;
          case 'h':
          case 'H': // hexa
            base = 16;
            i++;
            break;
          case 'b':
          case 'B': // binaire
            base = 2;
            i++;
            break;
          default:
            rc = -121; // Invalid character in number
            break;
        }
        break;
      default:
        break;
    }
    if (rc == 0)
    {
      if (s[i] == '\0')
      {
        rc = -120; // Numeric data error
      }
      else
      {
        char c;
        while ((rc == 0) && ((c = s[i++]) != '\0'))
        {
          int8_t digit = 99;
          if ((c >= '0') && (c <= '9'))
          {
            digit = c - '0';
          }
          else if ((c >= 'A') && (c <= 'F'))
          {
            digit = 10 + c - 'A';
          }
          else if ((c >= 'a') && (c <= 'f'))
          {
            digit = 10 + c - 'a';
          }
          if (digit >= base)
          {
            rc = -121; // Invalid character in number
          }
          if (rc == 0)
          {
            if (n <= ((INT32_MAX - digit)/base))
            {
              n *= base;
              n += digit;
            }
            else
            {
              rc = -124; // Too many digits
            }
          }
        }
      }
    }
    if (rc == 0)
    {
      value = negvalue?-n:n;
    }
  }
  return rc;
}

/**
 * @brief Interprète une chaîne comme un nombre flottant au format NRf (mantisse avec signe et partie décimale, exposant optionnel).
 * @param s Chaîne à interpréter.
 * @param value [out] Valeur flottante obtenue.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t isNRf(const char *s, float &value)
{
  int16_t rc = 0;
  float v = 0.0;
  if (s != NULL)
  {
    uint8_t idx = 0;
    size_t i = 0;
    bool mantisNeg = false;
    int16_t exponent = 0;
    float deci = 0.0;
    bool exponentNeg = true;
    int position = -1; // 0 = mantissa sign, 1 = mantissa (integer part), 2 = mantissa (dot), 3 = mantissa (decimal part), 4 = 'E', 5 = exponent sign, 6 exponent
    int digit = 0;
    float decimult = 1;

    switch (s[i])
    {
      case '+':
        // mantisNeg = false;
        position = 0;
        i++;
        break;
      case '-':
        position = 0;
        mantisNeg = true;
        i++;
        break;
      case '\0':
        rc = -120; // Numeric data error
        break;
      default:
        break;
    }
    if (rc == 0)
    {
      while (s[i] == '0')
      {
        position = 1;
        i++;
      }
      if ((s[i] == '\0') && (position < 1)) rc = -120; // Numeric data error (only + or - sign without digit after)
      char c;
      position = 1;
      while ((rc == 0) && ((c = s[i++]) != '\0'))
      {
        switch (c)
        {
          case 'E':
          case 'e':
            if ((position != 1) && (position != 3)) rc = -121; // Invalid character in number
            position = 4;
            break;
          case '.':
            if (position > 1) rc = -121; // Invalid character in number
            position = 2;
            break;
          case '+':
          case '-':
            if (position == -1)
            {
              position = 0;
            }
            else if (position == 4)
            {
              position = 5;
              exponentNeg = (c == '-');
            }
            else
            {
              rc = -121; // Invalid character in number
            }
            break;
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            digit = c - '0';
            switch (position)
            {
              case -1: // begin
              case 0: // sign
                position = 1;
                // no break
              case 1: // integer
                v *= 10;
                v += digit;
                break;
              case 2: // dot
                position = 3;
                // no break
              case 3: // decimal
                decimult /= 10.0;
                deci += (float)digit * decimult;
                break;
              case 4:
              case 5:
                position = 6;
                // no break;
              case 6:
                if (((exponent * 10) + digit) > 32000)
                {
                  rc = -123; // Exponent too large
                }
                else
                {
                  exponent *= 10;
                  exponent += digit;
                }
                break;
              default:
                rc = -310; // System error
                break;
            }
            if ((position == 1) || (position == 3))
            {
              idx++;
              if (idx == 0)
              {
                rc = -124; // Too many digits
                break;
              }
            }
            break;
          default:
            rc = -121; // Invalid character in number
            break;
        }
      }
    }
    if (rc == 0)
    {
      v = v + deci;
      if (mantisNeg) v = -v;
      if (exponentNeg) exponent = -exponent;
#ifdef DEBUG_PARSING
      Serial.print(" >> ");
      Serial.println(v);
#endif
      // TODO: tenir compte de de l'exposant
      if ((v < SCPI_NINFINITY) || (v > SCPI_PINFINITY))
      { // Limites 
        rc = -222; // Data out of range
      }
    }
  }
  if (rc == 0)
  {
    value = v;
  }
  return rc;
}

/**
 * @brief Compare le nombre d'arguments reçus au nombre attendu.
 * @param expected Nombre d'arguments attendu.
 * @param provided Nombre d'arguments effectivement fournis.
 * @return 0 si les comptes correspondent, -109 s'il manque des arguments, -108 s'il y en a trop.
 */
int16_t compareArgumentsCount(int expected, int provided)
{
  if (provided == expected)
  {
    return 0;
  }
  else if (provided < expected)
  {
    return -109;
  }
  else
  {
    return -108;
  }
}

/**
 * @brief Traite une commande ou requête SCPI à paramètre booléen (avec support du mot-clé DEFault).
 *
 * En mode commande, retourne la valeur analysée (qui peut être la valeur par défaut). En mode requête,
 * affiche la valeur fournie (par défaut ou courante).
 * @param isQuery true si c'est une requête (suffixe '?').
 * @param ac Nombre d'arguments.
 * @param av Tableau des arguments.
 * @param defValue Valeur par défaut associée au mot-clé DEFault.
 * @param value [in,out] Valeur courante (utilisée/affichée en requête) ; reçoit la nouvelle valeur en commande.
 * @param rc [in,out] Code d'erreur SCPI, mis à jour en cas de problème. Rien n'est fait si déjà non nul en entrée.
 * @return true si `value` doit être modifiée par le nouveau contenu.
 */
bool checkBoolean(bool isQuery, int ac, char *av[], bool defValue, bool &value, int16_t &rc)
{ // x? [DEFault] & x <boolean>|DEFault
  if (rc == 0)
  {
    bool v;
    if (isQuery)
    {
      switch (ac)
      {
        case 0:
          v = value;
          break;
        case 1:
          if (isDefault(av[0]))
          {
            v = defValue;
          }
          else
          {
            rc = -224; // Illegal parameter value
          }
          break;
        default:
          rc = -108; // Parameter not allowed
          break;
      }
      if (rc == 0)
      {
        displaySeparator();
        displayBoolean(v);
      }
    }
    else
    {
      rc = compareArgumentsCount(1, ac);
      if (rc == 0)
      {
        if (isDefault(av[0]))
        {
          v = defValue;
        }
        else
        {
          rc = isBool(av[0], v);
        }
      }
      if (rc == 0)
      {
        value = v;
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief Traite une commande ou requête SCPI à paramètre entier borné sur un octet (format NR1).
 * @param isQuery true si c'est une requête (suffixe '?').
 * @param ac Nombre d'arguments.
 * @param av Tableau des arguments.
 * @param min Valeur minimale autorisée.
 * @param max Valeur maximale autorisée.
 * @param value [in,out] Valeur courante (utilisée/affichée en requête) ; reçoit la nouvelle valeur en commande.
 * @param rc [in,out] Code d'erreur SCPI, mis à jour en cas de problème. Rien n'est fait si déjà non nul en entrée.
 * @return true si `value` doit être modifiée par le nouveau contenu.
 */
bool checkByte(bool isQuery, int ac, char*av[], const uint8_t min, const uint8_t max, uint8_t &value, int16_t &rc)
{ // x? & x <NR1>
  if (rc == 0)
  {
    if (isQuery)
    {
      displaySeparator();
      displayInteger(value);
    }
    else
    {
      rc = compareArgumentsCount(1, ac);
      if (rc == 0)
      {
        int32_t v = -1;
        rc = isNR1(av[0], v);
        if (rc == 0)
        {
          if ((v < min) || (v > max))
          {
            rc = -222;
          }
          else
          {
            value = v;
            return true;
          }
        }
      }
    }
  }
  return false;
}

/**
 * @brief Traite une commande ou requête SCPI à paramètre entier (format NR1), avec support des mots-clés MINimum/DEFault/MAXimum.
 * @param isQuery true si c'est une requête (suffixe '?').
 * @param ac Nombre d'arguments.
 * @param av Tableau des arguments.
 * @param valueMin Valeur minimale (associée au mot-clé MINimum).
 * @param valueDef Valeur par défaut (associée au mot-clé DEFault).
 * @param valueMax Valeur maximale (associée au mot-clé MAXimum).
 * @param value [in,out] Valeur courante (utilisée/affichée en requête) ; reçoit la nouvelle valeur en commande.
 * @param rc [in,out] Code d'erreur SCPI, mis à jour en cas de problème. Rien n'est fait si déjà non nul en entrée.
 * @return true si `value` doit être modifiée par le nouveau contenu.
 */
bool checkInteger(bool isQuery, int ac, char* av[], int32_t valueMin, int32_t valueDef, int32_t valueMax, int32_t &value, int16_t &rc)
{ // x? [MINimum|DEFault|MAXimum] & x <NR1>|MINimum|DEFault|MAXimum
  if (rc == 0)
  {
    int32_t v;
    if (isQuery)
    {
      switch (ac)
      {
        case 0:
          v = value;
          break;
        case 1:
          if (isDefault(av[0]))
          {
            v = valueDef;
          }
          else if (isMinimum(av[0]))
          {
            v = valueMin;
          }
          else if (isMaximum(av[0]))
          {
            v = valueMax;
          }
          else
          {
            rc = -224;
          }
          break;
        default:
          rc = -108;
          break;
      }
      if (rc == 0)
      {
        displaySeparator();
        displayInteger(v);
      }
    }
    else
    {
      rc = compareArgumentsCount(1, ac);
      if (rc == 0)
      {
        if (isDefault(av[0]))
        {
          v = valueDef;
        }
        else if (isMinimum(av[0]))
        {
          v = valueMin;
        }
        else if (isMaximum(av[0]))
        {
          v = valueMax;
        }
        else
        {
          rc = isNR1(av[0], v);
        }
      }
      if (rc == 0)
      {
        if ((v < valueMin) || (v > valueMax))
        {
          rc = -222;
        }
        else
        {
          value = v;
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * @brief Traite une commande ou requête SCPI à paramètre flottant (format NRf), avec support des mots-clés MINimum/DEFault/MAXimum.
 * @param isQuery true si c'est une requête (suffixe '?').
 * @param ac Nombre d'arguments.
 * @param av Tableau des arguments.
 * @param valueMin Valeur minimale (associée au mot-clé MINimum).
 * @param valueDef Valeur par défaut (associée au mot-clé DEFault).
 * @param valueMax Valeur maximale (associée au mot-clé MAXimum).
 * @param value [in,out] Valeur courante (utilisée/affichée en requête) ; reçoit la nouvelle valeur en commande.
 * @param rc [in,out] Code d'erreur SCPI, mis à jour en cas de problème. Rien n'est fait si déjà non nul en entrée.
 * @return true si `value` doit être modifiée par le nouveau contenu.
 */
bool checkFloat(bool isQuery, int ac, char* av[], float valueMin, float valueDef, float valueMax, float &value, int16_t &rc)
{ // x? [MINimum|DEFault|MAXimum] & x <NRf>|MINimum|DEFault|MAXimum
  if (rc == 0)
  {
    float v;
    if (isQuery)
    {
      switch (ac)
      {
        case 0:
          v = value;
          break;
        case 1:
          if (isDefault(av[0]))
          {
            v = valueDef;
          }
          else if (isMinimum(av[0]))
          {
            v = valueMin;
          }
          else if (isMaximum(av[0]))
          {
            v = valueMax;
          }
          else
          {
            rc = -224;
          }
          break;
        default:
          rc = -108;
          break;
      }
      if (rc == 0)
      {
        displaySeparator();
        displayFloat(v);
      }
    }
    else
    {
      rc = compareArgumentsCount(1, ac);
      if (rc == 0)
      {
        if (isDefault(av[0]))
        {
          v = valueDef;
        }
        else if (isMinimum(av[0]))
        {
          v = valueMin;
        }
        else if (isMaximum(av[0]))
        {
          v = valueMax;
        }
        else
        {
          rc = isNRf(av[0], v);
        }
      }
      if (rc == 0)
      {
        if ((v < valueMin) || (v > valueMax))
        {
          rc = -222;
        }
        else
        {
          value = v;
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * @brief Compacte une date (année/mois/jour) dans un mot de 16 bits (7 bits année depuis 2000, 4 bits mois, 5 bits jour).
 * @param year Année (2000-2127).
 * @param month Mois (1-12).
 * @param day Jour (1-31).
 * @return Date compactée, ou 0 si les valeurs sont hors plage.
 */
uint16_t dateToWord(int year, int month, int day)
{
  if ((year < 2000) || (year > 2127) || (month < 1) || (month > 12) || (day < 1) || (day > 31))
  {
    return 0;
  }
  else
  {
    return ((year - 2000) << 9) | (month << 5) | day;
  }
}

/**
 * @brief Lit la date courante depuis l'horloge temps réel (RTC) et la compacte au format interne 16 bits.
 * @return Date compactée, ou 0 si le RTC n'est pas accessible.
 */
uint16_t currentDateToWord()
{
  RTCTime currentTime;
  if (RTC.getTime(currentTime))
  {
    int currentYear = currentTime.getYear();
    int currentMonth = Month2int(currentTime.getMonth());
    int currentDay = currentTime.getDayOfMonth();
    return dateToWord(currentYear, currentMonth, currentDay);
  }
  else
  {
    return 0;
  }
}

/**
 * @brief Traite une commande ou requête SCPI portant sur une date (année, mois, jour), avec support des mots-clés MINimum/DEFault/MAXimum.
 * @param isQuery true si c'est une requête (suffixe '?').
 * @param ac Nombre d'arguments.
 * @param av Tableau des arguments.
 * @param value [in,out] Date compactée courante (utilisée/affichée en requête) ; reçoit la nouvelle date en commande.
 * @param rc [in,out] Code d'erreur SCPI, mis à jour en cas de problème. Rien n'est fait si déjà non nul en entrée.
 * @return true si `value` doit être modifiée par le nouveau contenu.
 */
bool checkDate(bool isQuery, int ac, char* av[], uint16_t &value, int16_t &rc)
{ // x? [MINimum|DEFault|MAXimum] & x <NRf>|MINimum|DEFault|MAXimum,<NRf>|MINimum|DEFault|MAXimum,<NRf>|MINimum|DEFault|MAXimum
  if (rc == 0)
  {
    uint16_t d = value;
    if (isQuery)
    {
      switch (ac)
      {
        case 0:
          d = value;
          break;
        case 1:
          if (isDefault(av[0]))
          {
            d = 0x0000;
          }
          else if (isMinimum(av[0]))
          {
            d = 0x0021; // 2000-01-01
          }
          else if (isMaximum(av[0]))
          {
            d = 0xFF9F; // 2127-12-31
          }
          else
          {
            rc = -224;
          }
          break;
        default:
          rc = -108;
          break;
      }
      if (rc == 0)
      {
        displaySeparator();
        displayDate(d);
      }
    }
    else
    {
      rc = compareArgumentsCount(3, ac);
      if (rc == 0)
      {
        int32_t a,m,j;
        a = (d & 0xFE00) >> 9;
        a += 2000;
        m = (d & 0x01E0) >> 5;
        j = d & 0x001F;
        if (checkInteger(false, 1, &av[0], 2000, a, 2127, a, rc)
         && checkInteger(false, 1, &av[1], 1, m, 12, m, rc)
         && checkInteger(false, 1, &av[2], 1, j, 31, j, rc))
        {
          value = dateToWord(a, m, j);
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * @brief Doit traiter une commande ou requête SCPI portant sur une heure (heure, minute, seconde), avec support
 * des mots-clés MINimum/DEFault/MAXimum, sur le même principe que checkDate().
 * @note Non implémentée (TODO) : le corps est actuellement vide.
 * @param isQuery true si c'est une requête (suffixe '?').
 * @param ac Nombre d'arguments.
 * @param av Tableau des arguments.
 * @param value [in,out] Heure compactée courante (utilisée/affichée en requête) ; reçoit la nouvelle heure en commande.
 * @param rc [in,out] Code d'erreur SCPI, mis à jour en cas de problème. Rien n'est fait si déjà non nul en entrée.
 * @return true si `value` doit être modifiée par le nouveau contenu.
 */
bool checkTime(bool isQuery, int ac, char* av[], uint16_t &value, int16_t &rc)
{ // x? [MINimum|DEFault|MAXimum] & x <NRf>|MINimum|DEFault|MAXimum,<NRf>|MINimum|DEFault|MAXimum,<NRf>|MINimum|DEFault|MAXimum
  // TODO
}

// ---------------------------------------------------------------------------
// Tendance de l'orage : bits 1 (approche) et 2 (s'eloigne) de QUES:LIGHtning
// Lissage : filtre median sur TREND_WINDOW mesures + bande morte TREND_HYST.
// ---------------------------------------------------------------------------
#define TREND_WINDOW  3   // taille de la fenetre du filtre median
#define TREND_HYST    2   // ecart minimal en km pour valider une tendance
                          // 1 = tout changement compte, 2 = ignore un cran

static uint8_t trendBuf[TREND_WINDOW];    // mesures brutes les plus recentes
static uint8_t trendCount;                // remplissage courant (0..TREND_WINDOW)
static uint8_t trendIndex;                // position d'ecriture circulaire
static uint8_t trendRef = UINT8_MAX;      // mediane de reference (dernier palier)

/**
 * @brief Réinitialise l'état du filtre de tendance de l'orage (mémoire tampon, compteur, valeur de référence).
 */
static void trendReset()
{
  trendCount = 0;
  trendIndex = 0;
  trendRef   = UINT8_MAX;
}

/**
 * @brief Calcule la médiane de trois valeurs sans tri ni division : max(min(a,b), min(max(a,b), c)).
 * @param a Première valeur.
 * @param b Deuxième valeur.
 * @param c Troisième valeur.
 * @return Médiane des trois valeurs.
 */
static uint8_t median3(uint8_t a, uint8_t b, uint8_t c)
{
  if (a > b) { uint8_t t = a; a = b; b = t; }  // a = min, b = max
  if (b > c) b = c;                            // b = min(max(a,b), c)
  return (a > b) ? a : b;
}

/**
 * @brief Met à jour la tendance de l'orage (se rapproche / s'éloigne, bits de QUEStionable:LIGHtning) à partir
 * de la distance courante, en lissant les mesures par un filtre médian avec bande morte (hystérésis).
 */
void updateStormTrend()
{
  // --- Pas d'orage exploitable : 0 = invalide, 63 = hors de portee ---
  if ((lightning_distance == 0) || (lightning_distance > DISTANCE_MAX))
  {
    regBitClear(lighReg, LIGH_SAPR);
    regBitClear(lighReg, LIGH_SLEA);
    regBitClear(lighReg, LIGH_STOR);
    regBitClear(lighReg, LIGH_SCLO);
    trendReset();
    return;
  }

  regBitSet(lighReg, LIGH_STOR);                       // orage en cours

  // --- Alimentation de la fenetre glissante ---
  trendBuf[trendIndex] = lightning_distance;
  if (++trendIndex >= TREND_WINDOW) trendIndex = 0;   // pas de modulo (division)
  if (trendCount < TREND_WINDOW) trendCount++;

  if (trendCount < TREND_WINDOW) return;       // fenetre incomplete

  uint8_t med = median3(trendBuf[0], trendBuf[1], trendBuf[2]);

  if (trendRef == UINT8_MAX)                   // premiere mediane : reference
  {
    trendRef = med;
    return;
  }

  if ((med + TREND_HYST) <= trendRef)
  {
    regBitSet(lighReg, LIGH_SAPR);                     // se rapproche
    regBitClear(lighReg, LIGH_SLEA);
    trendRef = med;
  }
  else if (med >= (trendRef + TREND_HYST))
  {
    regBitSet(lighReg, LIGH_SLEA);                     // s'eloigne
    regBitClear(lighReg, LIGH_SAPR);
    trendRef = med;
  }
  // Sinon : variation dans la bande morte, tendance precedente conservee
}

/**
 * @brief Affiche la dernière distance de l'orage mesurée, convertie en mètres (NaN si invalide, +infini si hors de portée).
 */
void fetchDistance()
{
  float f;
  switch (lightning_distance)
  {
    case UINT8_MAX:
      f = SCPI_NAN;
      break;
    case 63:
      f = SCPI_PINFINITY;
      break;
    default:
      f = 1000.0 * float(lightning_distance);
      break;
  }
  displayFloat(f);
}

/**
 * @brief Lit la distance de l'orage depuis le capteur AS3935, met à jour les statistiques générales et,
 * si un éclair a été détecté, les statistiques spécifiques à la foudre et la tendance de l'orage.
 * @param strike true si l'appel suit la détection d'un éclair (et non un simple changement de distance).
 */
void readDistance(bool strike)
{
  lightning_distance = lightning.distanceToStorm();
  int16_t rc = 0;
  if ((lightning_distance > 0) && (lightning_distance < 41))
  { // valid data and not out of range
    if (lightning_distance < distanceMinimum) distanceMinimum = lightning_distance;
    if (lightning_distance > distanceMaximum) distanceMaximum = lightning_distance;
    if ((UINT32_MAX - distanceTotal) < lightning_distance)
    {
      if (!isDistanceOvfl)
      {
        distanceTotal = UINT32_MAX;
        isDistanceOvfl = true;
        rc = -222; // Data out of range
      }
    }
    else
    {
      distanceTotal += lightning_distance;
    }
    if (distanceCount == UINT32_MAX)
    {
      if (!isDistanceCountOvfl)
      {
        isDistanceCountOvfl = true;
        rc = -222; // Data out of range
      }
    }
    else
    {
      distanceCount++;
    }
    if (strike)
    {
      updateStormTrend();
      if (lightning_distance < lightningDistanceMinimum) lightningDistanceMinimum = lightning_distance;
      if (lightning_distance > lightningDistanceMaximum) lightningDistanceMaximum = lightning_distance;
      if ((UINT32_MAX - lightningDistanceTotal) < lightning_distance)
      {
        if (!isLightningDistanceOvfl)
        {
          lightningDistanceTotal = UINT32_MAX;
          isLightningDistanceOvfl = true;
          rc = -222; // Data out of range
        }
      }
      else
      {
        lightningDistanceTotal += lightning_distance;
      }
    }
  }
  else if (lightning_distance == 63)
  { // strike too far away
    regBitClear(lighReg, LIGH_STOR); // Fin orage
    regBitClear(lighReg, LIGH_SCLO);
    regBitClear(lighReg, LIGH_SAPR);
    regBitClear(lighReg, LIGH_SLEA);
  }
  if (rc != 0)
  {
    pushError(rc);
  }
}

/**
 * @brief Affiche la dernière température mesurée.
 */
void fetchTemperature()
{
  displayFloat(temperature);
}

/**
 * @brief Lit la tension sur l'entrée analogique A0 et calcule la température par calibration linéaire deux points ;
 * met à jour les bits d'état de sur/sous-température de QUEStionable:TEMPerature.
 */
void readTemperature()
{
  if (bitRead(tempReg.con, TEMP_NCAL) || (cal.t_gain == 0.0))
  {
    temperature = SCPI_NAN;
  }
  else
  {
/*
    // Le capteur de température NÉCESSITE la référence interne 1.1V (REFS1:0 = 11)
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
    delay(20); // Attente de stabilisation (le capteur de temp est plus lent que Vcc, certains recommandent 20ms au 1er relevé)
    ADCSRA |= _BV(ADSC); // Démarrage conversion
    while (bit_is_set(ADCSRA, ADSC));

    uint8_t low  = ADCL;
    uint8_t high = ADCH;
    int raw = (high << 8) | low;
*/
    int raw = analogRead(A0);

    // Conversion via calibration linéaire deux points
    // Retourne la température en °C
    temperature = (float(raw) - cal.t_offset) / cal.t_gain;
    regBitWrite(tempReg, TEMP_UTP, (temperature <= 10)); // Under temperature
    regBitWrite(tempReg, TEMP_OTP, (temperature >= 50)); // Over temperature
  }
}

/**
 * @brief Affiche la dernière tension d'alimentation Vcc mesurée, en volts.
 */
void fetchVcc()
{
  float f = float(voltVcc) / 1000.0;
  if (voltVcc == UINT32_MAX)
  {
    f = SCPI_NAN;
  }
  displayFloat(f);
}

/**
 * @brief Mesure la tension d'alimentation Vcc via la référence interne 1.1V et met à jour les bits d'état
 * de sur/sous-tension de QUEStionable:VOLTage.
 */
void readVcc() {
/*
    // Lit la référence interne 1.1V par rapport à AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2); // Attente de stabilisation
    ADCSRA |= _BV(ADSC); // Démarrage conversion
    while (bit_is_set(ADCSRA, ADSC));
    
    uint8_t low  = ADCL;
    uint8_t high = ADCH;
    long result = (high << 8) | low;
*/
    long result = analogRead(A1);    
    // Calcul de Vcc en mV (1125300 = 1.1V * 1023 * 1000)
    result = 1125300L / result; // result est en mV (ex: 4980 pour 4.98V)
    regBitWrite(voltReg, VOLT_VCUV, (result <= VCC_MIN_MV)); // Under Voltage
    regBitWrite(voltReg, VOLT_VCOV, (result >= VCC_MAX_MV)); // Over voltage
    voltVcc = result;
}

/**
 * @brief Affiche la dernière énergie de foudre mesurée, en valeur brute ou en pourcentage de l'échelle maximale du capteur.
 * @param inPercent true pour afficher un pourcentage plutôt que la valeur brute.
 */
void fetchEnergy(bool inPercent)
{
  float f;
  if (lightning_energy == UINT32_MAX)
  {
    f = SCPI_NAN;
  }
  else if (inPercent)
  {
    f = 100.0 * (double)lightning_energy / (double)ENERGIE_MAX;
  }
  else
  {
    f = lightning_energy;
  }
  displayFloat(f);
}

/**
 * @brief Lit l'énergie de la dernière détection depuis le capteur AS3935, met à jour les statistiques générales et,
 * si un éclair a été détecté, les statistiques spécifiques à la foudre.
 * @param strike true si l'appel suit la détection d'un éclair (et non un simple changement de distance).
 */
void readEnergy(bool strike)
{
  lightning_energy = lightning.lightningEnergy();
  int16_t rc = 0;
  if (lightning_energy < energyMinimum) energyMinimum = lightning_energy;
  if (lightning_energy > energyMaximum) energyMaximum = lightning_energy;
  if ((UINT32_MAX - energyTotal) < lightning_energy)
  {
    if (!isEnergyOvfl)
    {
      energyTotal = UINT32_MAX;
      isEnergyOvfl = true;
      rc = -222; // Data out of range
    }
  }
  else
  {
    energyTotal += lightning_energy;
  }
  if (energyCount == UINT32_MAX)
  {
    if (!isEnergyCountOvfl)
    {
      isEnergyCountOvfl = true;
      rc = -222; // Data out of range
    }
  }
  else
  {
    energyCount++;
  }
  if (strike)
  {
    if (lightning_energy < lightningEnergyMinimum) lightningEnergyMinimum = lightning_energy;
    if (lightning_energy > lightningEnergyMaximum) lightningEnergyMaximum = lightning_energy;
    if ((UINT32_MAX - lightningEnergyTotal) < lightning_energy)
    {
      if (!isLightningEnergyOvfl)
      {
        lightningEnergyTotal = UINT32_MAX;
        isLightningEnergyOvfl = true;
        rc = -222; // Data out of range
      }
    }
    else
    {
      lightningEnergyTotal += lightning_energy;
    }
  }
  if (rc != 0)
  {
    pushError(rc);
  }
}

/**
 * @brief Affiche le type du dernier événement détecté (bruit, parasite, éclair), en forme textuelle ou en code numérique brut.
 * @param text true pour un affichage textuel (ex: "LIGH"), false pour le code numérique brut.
 */
void fetchType(bool text)
{
  if (lightning_intReason == UINT8_MAX)
  {
    displayFloat(SCPI_NAN);
  }
  else
  {
    if (text)
    {
      switch (lightning_intReason)
      {
        case TOO_NOISY_INT:
          Serial.print("NOIS");
          break;
        case DISTURBER_INT:
          Serial.print("DIST");
          break;
        case LIGHTNING_INT:
          Serial.print("LIGH");
          break;
        case 0:
          Serial.print("DUPD");
          break;
        default:
          displayInteger(lightning_intReason);
          break;
      }
      scpiOutput = true;
    }
    else
    {
      displayInteger(lightning_intReason);
    }
  }
}

/**
 * @brief Analyse et exécute une commande ou requête SCPI standard IEEE 488.2 (préfixée par '*' : *CAL, *CLS,
 * *ESE, *ESR, *IDN, *OPC, *PSC, *PUD, *RCL, *RST, *SAV, *SRE, *STB, *TST, *WAI).
 * @param command Mnémonique de la commande standard (sans le '*').
 * @param isQuery true si c'est une requête (suffixe '?').
 * @param argc Nombre d'arguments fournis.
 * @param argv Tableau des arguments.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t processSCPIStandardCommand(char *command, bool isQuery, int argc, char *argv[])
{
  int16_t rc = 0;
  if (isQuery && (argc > 0))
  { // Here queries cannot have arguments (it could be possible in general)
    rc = -108;
  }
  else if (isQuery && isToken(command, F("CAL")))
  { // Auto-calibration (*CAL?)
    uint8_t cal = 0;
    rc = selfCalibration(cal);
    displaySeparator();
    displayInteger(cal);
  }
  else if (!isQuery && isToken(command, F("CLS")))
  { // Clear status (*CLS) [MAND]
    if (argc == 0)
    {
      clearErrorQueue();
      regClearEvents();
    }
    else
    {
      rc = -108;
    }
  }
  else if (isToken(command, F("ESE")))
  { // Event Status Enable register (*ESE? & *ESE <NR1>) [MAND]
    uint8_t v = scpiEse;
    if (checkByte(isQuery, argc, argv, 0x00, 0xFF, v, rc))
    {
          scpiEse = v;
          updateStb();
          savePowerOnStatus();
    }
  }
  else if (isQuery && isToken(command, F("ESR")))
  { // Event Status Register (*ESR?) [MAND]
    displaySeparator();
    displayInteger(scpiEsr);
    scpiEsr = 0x00;
    updateStb();
  }
  else if (isQuery && isToken(command, F("IDN")))
  { // Identification (*IDN?) [MAND]
    displaySeparator();
    // Fabricant,modèle,numéro de série,version du firmware
    Serial.print(F("ValTronix,LightningDetector,"));
    displaySerial();
    Serial.write(',');
#ifndef FIRMWARE_VERSION
    Serial.print(buildDateISO);
#else
    Serial.print(FIRMWARE_VERSION);
#endif
  }
  else if (isToken(command, F("OPC")))
  { // Operation Complete Command (*OPC? & *OPC) [MAND]
    if (isQuery)
    {
      displaySeparator();
      displayBoolean(true); // No backgound operation in progress
    }
    else
    {
      if (argc == 0)
      {
        bitSet(scpiEsr, 0);
        updateStb();
      }
      else
      {
        rc = -108;
      }
    }
  }
#ifdef SCPI_PSC
  else if (isToken(command, F("PSC")))
  { // Power-On Status Clear (*PSC? & *PSC <NR1>)
    bool b = isPSC;
    if (checkBoolean(isQuery, argc, argv, true, b, rc))
    {
      isPSC = b;
      savePowerOnStatus();
    }
  }
#endif
#ifdef SCPI_PUD
  else if (isToken(command, F("PUD")))
  { // Protected User Data (*PUD? & *PUD {<Block>|<QString>})
    if (isQuery)
    {
      SavedPud pud;
      rc = loadProtectedUsedData(pud);
      if (rc == 0)
      {
        displaySeparator();
        Serial.write('#');
        uint8_t l = pud.len;
        uint8_t i = 1;
        while (l >= 10)
        {
          i++;
          l /= 10;
        }
        Serial.print(i);
        Serial.print(pud.len);
        Serial.write((uint8_t*)pud.data, pud.len);
        scpiOutput = true;
      }
      else
      {
        rc = -312; // PUD memory lost
      }
    }
    else
    {
      rc = compareArgumentsCount(1, argc);
      if ((rc == 0) && isProtected)
      {
        rc = -203; // Command Protected
      }
      if (rc == 0)
      {
        rc = saveProtectedUsedData(argv[0]);
      }
    }
  }
#endif
  else if (!isQuery && isToken(command, F("RCL")))
  { // Recall instrument State (*RCL <NR1>)
    uint8_t slot = 0xFF;
    if (checkByte(isQuery, argc, argv, 0, SLOT_COUNT - 1, slot, rc))
    {
      rc = recallState(slot);
    }
  }
  else if (!isQuery && isToken(command, F("RST")))
  { // Reset to default settings (*RST) [MAND]
    if (argc == 0)
    {
      resetSettings();
    }
    else
    {
      rc = -108;
    }
  }
  else if (!isQuery && isToken(command, F("SAV")))
  { // Save instrument State (*SAV <NR1>)
    uint8_t slot = 0xFF;
    if (checkByte(isQuery, argc, argv, 0, SLOT_COUNT - 1, slot, rc))
    {
      rc = saveState(slot);
    }
  }
  else if (isToken(command, F("SRE")))
  { // Service Request Enable register (*SRE? & *SRE <NR1>) [MAND]
    uint8_t v = scpiSre;
    if (checkByte(isQuery, argc, argv, 0x00, 0xFF, v, rc))
    {
          scpiSre = v & 0xBF; // bit 6 = 0
          updateStb();
          savePowerOnStatus();
    }
  }
  else if (isQuery && isToken(command, F("STB")))
  { // Status Byte register (*STB?) [MAND]
    updateStb();
    displaySeparator();
    displayInteger(scpiStb);
    scpiOutput = true;
  }
  else if (isQuery && isToken(command, F("TST")))
  { // Self-test (*TST?) [MAND]
    uint8_t testResult = 0;
    rc = autoTest(testResult);
    displaySeparator();
    displayInteger(testResult);
  }
  else if (!isQuery && isToken(command, F("WAI")))
  { // Wait all background operation to complete (*WAI) [MAND]
    rc = compareArgumentsCount(0, argc);
    // No background operation exists, so nothing to wait for
  }
  else
  { // Invalid header
    rc = -113;
  }
  return rc;
}

/**
 * @brief Analyse et exécute une commande ou requête SCPI spécifique à l'appareil (arborescence ':' :
 * CALCulate, CALibration, DIAGnostic, FETCh, INITiate, MEASure, SENSe, STATus, SYSTem).
 * @param command Chaîne de l'en-tête de commande (arborescence ':' complète, sans le premier ':').
 * @param isQuery true si c'est une requête (suffixe '?').
 * @param argc Nombre d'arguments fournis.
 * @param argv Tableau des arguments.
 * @return 0 en cas de succès, sinon un code d'erreur SCPI.
 */
int16_t processSCPISpecificCommand(char *command, bool isQuery, int argc, char *argv[])
{
  int16_t rc = 0;
  char *token = strtok(command, ":");
  char *subtoken = NULL;
  if (isToken(token, F("CALCulate")))
  { // :CALCulate
    subtoken = nextToken();
    if (isToken(subtoken, F("LIGHtning")))
    { // :CALCulate:LIGHtning
      subtoken = nextToken();
      if (((subtoken == NULL) || isCount(subtoken)) && isQuery)
      { // :CALCulate:LIGHtning[:COUNt]?
        rc = compareArgumentsCount(0, argc);
        if (rc == 0)
        {
          displaySeparator();
          displayInteger(lightningCount);
        }
      }
      else if (isClear(subtoken) && !isQuery)
      { // :CALCulate:LIGHtning:CLEar
        rc = compareArgumentsCount(0, argc);
        if (rc == 0)
        {
          resetStatistics(STAT_LIGHTNING);
        }
      }
      else if (isToken(subtoken, F("ENERgy")))
      { // :CALCulate:LIGHtning:ENERgy
        subtoken = nextToken();
        bool inPercent = false;
        if (isPercent(subtoken))
        { // :CALCulate:LIGHtning:ENERgy:PERCent
          inPercent = true;
          subtoken = nextToken();
        }
        rc = compareArgumentsCount(0, argc);
        if (isQuery && isMinimum(subtoken))
        { // :CALCulate:LIGHtning:ENERgy[:PERCent]:MINimum?
          if (rc == 0)
          {
            float f;
            if (lightningCount == 0)
            {
              f = SCPI_NAN;
            }
            else
            {
              if (inPercent)
              {
                f = 100.0 * (double)lightningEnergyMinimum/(double)ENERGIE_MAX;
              }
              else
              {
                f = lightningEnergyMinimum;
              }
            }
            displaySeparator();
            displayFloat(f);
          }
        }
        else if (isQuery && isMaximum(subtoken))
        { // :CALCulate:LIGHtning:ENERgy[:PERCent]:MAXimum?
          if (rc == 0)
          {
            float f;
            if (lightningCount == 0)
            {
              f = SCPI_NAN;
            }
            else
            {
              if (inPercent)
              {
                f = 100.0 * (double)lightningEnergyMaximum/(double)ENERGIE_MAX;
              }
              else
              {
                f = lightningEnergyMaximum;
              }
            }
            displaySeparator();
            displayFloat(f);
          }
        }
        else if (isQuery && isAverage(subtoken))
        { // :CALCulate:LIGHtning:ENERgy[:PERCent]:AVERage?
          if (rc == 0)
          {
            float f;
            if ((lightningCount == 0) || isLightningEnergyOvfl || isLightningCountOvfl)
            {
              f = SCPI_NAN;
            }
            else
            {
              double average = (double)lightningEnergyTotal / (double)lightningCount;
              if (inPercent)
              {
                f = 100.0 * average/(double)ENERGIE_MAX;
              }
              else
              {
                f = average;
              }
            }
            displaySeparator();
            displayFloat(f);
          }
        }
        else
        {
          rc = -113;
        }
      }
      else if (isToken(subtoken, F("DISTance")))
      { // :CALCulate:LIGHtning:DISTance
        subtoken = nextToken();
        rc = compareArgumentsCount(0, argc);
        if (isQuery && isMinimum(subtoken))
        { // :CALCulate:LIGHtning:DISTance:MINimum?
          if (rc == 0)
          {
            float f;
            if (lightningCount == 0)
            {
              f = SCPI_NAN;
            }
            else
            {
              f = 1000.0 * float(lightningDistanceMinimum);
            }
            displaySeparator();
            displayFloat(f);
          }
        }
        else if (isQuery && isMaximum(subtoken))
        { // :CALCulate:LIGHtning:DISTance:MAXimum?
          if (rc == 0)
          {
            float f;
            if (lightningCount == 0)
            {
              f = SCPI_NAN;
            }
            else
            {
              f = 1000.0 * float(lightningDistanceMaximum);
            }
            displaySeparator();
            displayFloat(f);
          }
        }
        else if (isQuery && isAverage(subtoken))
        { // :CALCulate:LIGHtning:DISTance:AVERage?
          if (rc == 0)
          {
            float f;
            if ((lightningCount == 0) || isLightningDistanceOvfl || isLightningCountOvfl)
            {
              f = SCPI_NAN;
            }
            else
            { // Solution pour éviter un débordement possible. Correspond à faire (1000UL * lightningDistanceTotal) / lightningCount
              f = ((1000UL * (lightningDistanceTotal / lightningCount))
                + ((1000UL * (lightningDistanceTotal % lightningCount)) / lightningCount));
            }
            displaySeparator();
            displayFloat(f);
          }
        }
        else
        {
          rc = -113;
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (isToken(subtoken, F("ENERgy")))
    { // :CALCulate:ENERgy
      subtoken = nextToken();
      if (((subtoken == NULL) || isCount(subtoken)) && isQuery)
      { // :CALCulate:ENERgy[:COUNt]?
        rc = compareArgumentsCount(0, argc);
        if (rc == 0)
        {
          displaySeparator();
          displayInteger(energyCount);
        }
      }
      else if (isClear(subtoken) && !isQuery)
      { // :CALCulate:ENERgy:CLEar
        rc = compareArgumentsCount(0, argc);
        if (rc == 0)
        {
          resetStatistics(STAT_ENERGY);
        }
      }
      else
      {
        bool inPercent = false;
        if (isPercent(subtoken))
        { // :CALCulate:ENERgy:PERCent
          inPercent = true;
          subtoken = nextToken();
        }
        rc = compareArgumentsCount(0, argc);
        if (rc == 0)
        {
          if (isQuery && isMinimum(subtoken))
          { // :CALCulate:ENERgy[:PERCent]:MINimum?
            float f;
            if (energyCount == 0)
            {
              f = SCPI_NAN;
            }
            else
            {
              if (inPercent)
              {
                f = 100.0 * (double)energyMinimum/(double)ENERGIE_MAX;
              }
              else
              {
                f = energyMinimum;
              }
            }
            displaySeparator();
            displayFloat(f);
          }
          else if (isQuery && isMaximum(subtoken))
          { // :CALCulate:ENERgy[:PERCent]:MAXimum?
            float f;
            if (energyCount == 0)
            {
              f = SCPI_NAN;
            }
            else
            {
              if (inPercent)
              {
                f = 100.0 * (double)energyMaximum/(double)ENERGIE_MAX;
              }
              else
              {
                f = energyMaximum;
              }
            }
            displaySeparator();
            displayFloat(f);
          }
          else if (isQuery && isAverage(subtoken))
          { // :CALCulate:ENERgy[:PERCent]:AVERage?
            float f;
            if ((energyCount == 0) || isEnergyOvfl || isEnergyCountOvfl)
            {
              f = SCPI_NAN;
            }
            else
            {
              double average = (double)energyTotal / (double)energyCount;
              if (inPercent)
              {
                f = 100.0 * average/(double)ENERGIE_MAX;
              }
              else
              {
                f = average;
              }
            }
            displaySeparator();
            displayFloat(f);
          }
          else
          {
            rc = -113;
          }
        }
      }
    }
    else if (isToken(subtoken, F("DISTurber")))
    { // :CALCulate:DISTurber
      subtoken = nextToken();
      if (((subtoken == NULL) || isCount(subtoken)) && isQuery)
      { // :CALCulate:DISTurber[:COUNt]?
        rc = compareArgumentsCount(0, argc);
        if (rc == 0)
        {
          displaySeparator();
          displayInteger(disturberCount);
        }
      }
      else if (isClear(subtoken) && !isQuery)
      { // :CALCulate:DISTurber:CLEar
        rc = compareArgumentsCount(0, argc);
        if (rc == 0)
        {
          resetStatistics(STAT_DISTURBER);
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (isToken(subtoken, F("NOISe")))
    { // :CALCulate:NOISe
      subtoken = nextToken();
      if (((subtoken == NULL) || isCount(subtoken)) && isQuery)
      { // :CALCulate:NOISe[:COUNt]?
        rc = compareArgumentsCount(0, argc);
        if (rc == 0)
        {
          displaySeparator();
          displayInteger(noiseCount);
        }
      }
      else if (isClear(subtoken) && !isQuery)
      { // :CALCulate:NOISe:CLEar
        rc = compareArgumentsCount(0, argc);
        if (rc == 0)
        {
          resetStatistics(STAT_NOISE);
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (!isQuery && isClear(subtoken))
    { // :CALCulate:CLEar
      rc = compareArgumentsCount(0, argc);
      if (rc == 0)
      {
        resetStatistics(STAT_ALL);
      }
    }
    else
    {
      rc = -113;
    }
  }
  else if (isToken(token, F("CALibration")))
  { // :CALibration
    subtoken = nextToken();
    if (isQuery && isCount(subtoken))
    { // :CALibration:COUNt?
      if (argc == 0)
      {
        displaySeparator();
        displayInteger(cal.count);
      }
      else
      {
        rc = -108; // Parameter not allowed
      }
    }
    else if (isToken(subtoken, F("SECure")))
    { // :CALibration:SECure
      subtoken = nextToken();
      if (isState(subtoken))
      { // :CALibration:SECure:STATe? & :CALibration:SECure:STATe <boolean>,<password>
        if (isQuery)
        {
          displaySeparator();
          displayBoolean(isProtected);
        }
        else
        {
          bool toProtect = true;
          bool pwdmatch = false;
          switch (argc)
          {
            case 0:
              rc = -109; // Missing parameter 
              break;
            case 1:
              rc = isBool(argv[0], toProtect);
              if (rc == 0)
              {
                if (toProtect)
                {
                  pwdmatch = true;
                }
                else
                {
                  rc = -109; // Missing parameter 
                }
              }
              break;
            case 2:
              rc = isBool(argv[0], toProtect);
              pwdmatch = (strncmp(cal.passwd, argv[1], 12) == 0) && (strlen(argv[1]) <= 12);
              break;
            default:
              rc = -108; // Parameter not allowed
              break;
          }
          if (rc == 0)
          {
            isProtected = toProtect || !pwdmatch;
            if (!pwdmatch) rc = -221; // Settings conflict
            if (isProtected && (sourceCal != CAL_SOUR_NONE))
            {
              sourceCal = CAL_SOUR_NONE;
              lightning.displayOscillator(sourceCal);
            }
          }
        }
      }
      else if (isToken(subtoken, F("CODE")) && !isQuery)
      { // :CALibration:SECure:CODE <password>
        rc = compareArgumentsCount(1, argc);
        if (rc == 0)
        {
          if (isProtected)
          {
            rc = -203; // Command protected
          }
          else
          { // Unquoted string up to 12 characters
            // Must start with letter (A-Z)
            // May contain letters, numbers (0-9) and underscores
            // TODO
            rc = saveCalibration(true);
          }
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (isToken(subtoken, F("CAPacitance")))
    {
      subtoken = nextToken();
      if (subtoken == NULL)
      { // :CALibration:CAPacitance? [MIN|MAX|DEF] &  :CALibration:CAPacitance <NR1>|MIN|MAX|DEF|UP|DOWN
        int32_t cap = lightning.readTuneCap();
        if (checkInteger(isQuery, argc, argv, 0, 0, 15, cap, rc))
        {
          if (isProtected)
          {
            rc = -203; // Command protected
          }
          else
          {
            lightning.tuneCap(cap);
          }
        }
      }
      else if (isQuery && isToken(subtoken, F("VALue")))
      { // :CALibration:CAPacitance:VALue?
        if (argc == 0)
        {
          uint8_t cap = lightning.readTuneCap();
          displaySeparator();
          displayFloat(float(cap) * 8E-12);
        }
        else
        {
          rc = -108; // Parameter not allowed
        }
      }
    }
    else if (isToken(subtoken, F("STRing")))
    { // :CALibration:STRing? & :CALibration:STRing "<string>"
      if (isQuery)
      {
        if (argc == 0)
        {
          displaySeparator();
          displayString(calString, sizeof(calString));
        }
        else
        {
          rc = -108; // Parameter not allowed
        }
      }
      else
      {
        rc = compareArgumentsCount(1, argc);
        if (rc == 0)
        {
          uint8_t i = 0;
          uint8_t j = 0;
          char *s = argv[0];
          char cs = s[i++];
          bool ok = true;
          bool inStr = false;
          if ((cs == '"') || (cs == '\''))
          {
            inStr = true;
          }
          else
          {
            rc = -151;
          }
          while (ok && (rc == 0))
          {
            char c = s[i++];
            if (c == '\0')
            {
              if (inStr)
              {
                rc = -151;
              }
              else
              {
                ok = false;
                calString[j++] = c;
              }
            }
            else if (c == cs)
            {
              inStr = !inStr;
              if (inStr)
              {
                calString[j++] = c;
              }
            }
            else if (inStr)
            {
              calString[j++] = c;
            }
            else
            {
              rc = -151;
            }
            if ((rc == 0) && (j > sizeof(calString)))
            {
              rc = -151;
            }
          }
        }
      }
    }
    else if (isQuery && isToken(subtoken, F("TEMPerature")))
    { // :CALibration:TEMPerature?
      if (argc == 0)
      {
        displaySeparator();
        displayFloat(cal.caltemp);
      }
      else
      {
        rc = -108; // Parameter not allowed
      }
    }
    else if (!isQuery && isToken(subtoken, F("AUTO")))
    { // :CALibration:AUTO
      // TODO
    }
    else if (isToken(subtoken, F("DIVider")))
    { // :CALibration:DIVider? & :CALibration:DIVider 16|32|64|128
      int32_t div = lightning.readDivRatio();
      if (checkInteger(isQuery, argc, argv, 16, 16, 128, div, rc))
      {
        if ((div == 16) || (div == 32) || (div == 64) || (div == 128))
        {
          lightning.changeDivRatio(div);
        }
        else
        {
          rc = -224; // Illegal parameter value
        }
      }
    }
    else if (isToken(subtoken, F("SOURce")))
    { // :CALibration:SOURce? & :CALibration:SOURce LCO|SRCO|TRCO|NONE
      if (isQuery)
      {
        displaySeparator();
        switch (sourceCal)
        {
          case CAL_SOUR_NONE:
            Serial.print(F("NONE"));
            break;
          case CAL_SOUR_TRCO:
            Serial.print(F("TRCO"));
            break;
          case CAL_SOUR_SRCO:
            Serial.print(F("SRCO"));
            break;
          case CAL_SOUR_LCO:
            Serial.print(F("LCO"));
            break;
          default:
            Serial.print(sourceCal);
            break;
        }
        scpiOutput = true;
      }
      else
      {
        rc = compareArgumentsCount(1, argc);
        if (rc == 0)
        {
          uint8_t newSrc = 0xFF;
          if (isToken(argv[0], F("NONE")))
          {
            newSrc = CAL_SOUR_NONE;
          }
          else if (isToken(argv[0], F("TRCO")))
          {
            newSrc = CAL_SOUR_TRCO;
          }
          else if (isToken(argv[0], F("SRCO")))
          {
            newSrc = CAL_SOUR_SRCO;
          }
          else if (isToken(argv[0], F("LCO")))
          {
            newSrc = CAL_SOUR_LCO;
          }
          else
          {
            rc = -224; // Illegal parameter value
          }
          if (rc == 0)
          {
            if (isProtected)
            {
              rc = -203;
            }
            else
            {
              lightning.displayOscillator(newSrc);
              sourceCal = newSrc;
            }
          }
        }
      }

    }
    else if (isToken(subtoken, F("FREQuency")))
    { // :CALibration:FREQuency
      if (isQuery)
      {
        int32_t freq = (cal.capafreq[0] << 16) + (cal.capafreq[1] << 8) + cal.capafreq[2];
        freq &= 0x000FFFFF;
        displaySeparator();
        displayFloat(float(freq));
      }
      // TODO
    }
    else if (!isQuery && isToken(subtoken, F("STORe")))
    { // :CALibration:STORe
      rc = saveCalibration(false);
    }
    else if (isToken(subtoken, F("DATE")))
    { // :CALibration:DATE? & :CALibration:DATE <NR1>,<NR1>,<NR1>
      uint16_t cdate = calibrationDate;
      if (checkDate(isQuery, argc, argv, cdate, rc))
      {
        if (isProtected)
        {
          rc = -203;
        }
        else
        {
          calibrationDate = cdate;
        }
      }
    }
    else if (isToken(subtoken, F("NDUE")))
    { // :CALibration:NDUE? & :CALibration:NDUE <NR1>,<NR1>,<NR1>
      uint16_t cdate = calibrationDueDate;
      if (checkDate(isQuery, argc, argv, cdate, rc))
      {
        if (isProtected)
        {
          rc = -203;
        }
        else
        {
          calibrationDueDate = cdate;
        }
      }
    }
    else
    {
      rc = -113;
    }
  }
  else if (isToken(token, F("DIAGnostic")))
  { // :DIAGnostic
    subtoken = nextToken();
    if (isToken(subtoken, F("REGister")))
    { // :DIAGnostic:REGister? <NR1> & :DIAGnostic:REGister <NR1>,<NR1>
      rc = compareArgumentsCount(isQuery?1:2, argc);
      if (rc == 0)
      {
        int32_t regaddr = -1, regvalue = -1;
        rc = isNR1(argv[0], regaddr);
        if ((regaddr < 0x00) || (regaddr > 0x3F))
        {
          rc = -222; // Data out of range
        }
        if ((rc == 0) && (argc > 1))
        {
          rc = isNR1(argv[1], regvalue);
          if ((regvalue < 0x00) || (regvalue > 0xFF))
          {
            rc = -222;
          }
        }
        if (rc == 0)
        {
          if (isQuery)
          {
            regvalue = lightning.readRegister(regaddr);
            displaySeparator();
            displayInteger(regvalue);
          }
          else if (isProtected)
          {
            rc = -203; // Command protected
          }
          else
          {
            lightning.writeRegister(regaddr, regvalue);
          }
        }
      }
    }
    else if (isToken(subtoken, F("LIGHtning")) && !isQuery)
    { // :DIAGnostic:LIGHtning <NR1>
      rc = compareArgumentsCount(1, argc);
      if (rc == 0)
      {
        int32_t dist = 0;
        if (isInfinity(argv[0]))
        {
          dist = 63;
        }
        else if (isMaximum(argv[0]))
        {
          dist = 40;
        }
        else if (isMinimum(argv[0]))
        {
          dist = 1;
        }
        else
        {
          rc = isNR1(argv[0], dist);
          if ((rc == 0) && ((dist < 1) || (dist > 40)))
          {
            rc = -222;
          }
        }
        if (rc == 0)
        {
          // readDistance(true);
          lightning_distance = dist;
          readEnergy(true);
          lightningDetected();
        }
      }
    }
    else
    {
      rc = -113;
    }
  }
  else if (isToken(token, F("FETCh")))
  { // :FETCh
    if (isQuery)
    {
      rc = compareArgumentsCount(0, argc);
      subtoken = nextToken();
      if (isToken(subtoken, F("DISTance")))
      { // :FETCh:DISTance?
        if (rc == 0)
        {
          displaySeparator();
          fetchDistance();
        }
      }
      else if (isToken(subtoken, F("ENERgy")))
      { // :FETCh:ENERgy[:PERCent]?
        bool inPercent = false;
        if (rc == 0)
        {
          subtoken = nextToken();
          if (subtoken == NULL)
          {
            inPercent = false;
          }
          else if (isPercent(subtoken))
          {
            inPercent = true;
          }
          else
          {
            rc = -113;
          }
        }
        if (rc == 0)
        {
          displaySeparator();
          fetchEnergy(inPercent);
        }
      }
      else if (isToken(subtoken, F("TYPE")))
      { // :FETCh:TYPE[:CODE]?
        subtoken = nextToken();
        if ((subtoken == NULL) || isToken(subtoken, F("CODE")))
        {
          if (rc == 0)
          {
            displaySeparator();
            fetchType(subtoken == NULL);
          }
        }
        else
        {
          rc = -113;
        }
      }
      else if (isToken(subtoken, F("TEMPerature")))
      { // :FETCh:TEMPerature?
        displaySeparator();
        fetchTemperature();
      }
      else if (isToken(subtoken, F("VOLTage")))
      { // :FETCh:VOLTage?
        displaySeparator();
        fetchVcc();
      }
      else
      {
        rc = -113;
      }
    }
    else
    {
      rc = -113;
    }
  }
  else if (isToken(token, F("INITiate")))
  { // :INITiate
    subtoken = nextToken();
    if (false)
    {
      // TODO
    }
    else
    {
      rc = -113;
    }
  }
  else if (isToken(token, F("MEASure")))
  { // :MEASure
    rc = compareArgumentsCount(0, argc);
    if ((rc == 0) && isQuery)
    {
      subtoken = nextToken();
      if (isToken(subtoken, F("DISTance")))
      { // :MEASure:DISTance?
        readDistance(false);
        displaySeparator();
        fetchDistance();
      }
      else if (isToken(subtoken, F("ENERgy")))
      { // :MEASure:ENERgy?
        readEnergy(false);
        displaySeparator();
        fetchEnergy(false);
      }
      else if (isToken(subtoken, F("ALL")))
      { // :MEASure:ALL?
        readDistance(false);
        readEnergy(false);
        displaySeparator();
        fetchDistance();
        Serial.write(',');
        fetchEnergy(false);
      }
      else if (isToken(subtoken, F("VOLTage")))
      { // :MEASure:VOLTage?
        subtoken = nextToken();
        if ((subtoken == NULL) || isToken(subtoken, F("DC")))
        { // :MEASure:VOLTage[:DC]?
          readVcc();
          displaySeparator();
          displayFloat(voltVcc);
        }
        else
        {
          rc = -113;
        }
      }
      else if (isToken(subtoken, F("TEMPerature")))
      { // :MEASure:TEMPerature?
        readTemperature();
        displaySeparator();
        fetchTemperature();
      }
      else
      {
        rc = -113;
      }
    }
    else if (rc == 0)
    {
      rc = -113;
    }
  }
  else if (isToken(token, F("SENSe")))
  { // :SENSe
    subtoken = nextToken();
    if (isToken(subtoken, F("AFE")))
    { // :SENSe:AFE
      subtoken = nextToken();
      if ((subtoken == NULL) || isToken(subtoken, F("GAIN")))
      { // :SENSe:AFE[:GAIN]? [DEFault] & :SENSe:AFE[:GAIN] INDoor|OUTdoor|DEFault (Registre AFE_GB)
        if (isAs3935Available)
        {
          int afe = 0;
          if (isQuery)
          {
            switch (argc)
            {
              case 0:
                afe = lightning.readIndoorOutdoor();
                switch (afe)
                {
                  case INDOOR:
                    displaySeparator();
                    Serial.print(F("IND"));
                    scpiOutput = true;
                    break;
                  case OUTDOOR:
                    displaySeparator();
                    Serial.print(F("OUT"));
                    scpiOutput = true;
                    break;
                  default:
                    rc = -224;
                    break;
                }
                break;
              case 1:
                if (isDefault(argv[0]))
                {
                  displaySeparator();
                  Serial.print(F("IND"));
                  scpiOutput = true;
                }
                else
                {
                  rc = -108;
                }
                break;
              default:
                rc = -108;
                break;
            }
          }
          else
          {
            rc = compareArgumentsCount(1, argc);
            if (rc == 0)
            {
              if (isDefault(argv[0]) || isToken(argv[0], F("INDoor")))
              {
                afe = INDOOR;
              }
              else if (isToken(argv[0], F("OUTdoor")))
              {
                afe = OUTDOOR;
              }
              else
              {
                rc = -224;
              }
              if (rc == 0)
              {
                lightning.setIndoorOutdoor(afe);
                if (lightning.readIndoorOutdoor() != afe)
                {
                  rc = -240;
                }
              }
            }
          }
        }
        else
        {
          rc = -241;
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (isToken(subtoken, F("NOISe")))
    { // :SENSe:NOISe
      subtoken = nextToken();
      if (isThreshold(subtoken))
      { // :SENSe:NOISe:THReshold? [MINimum|DEFault|MAXimum] & :SENSe:NOISe:THReshold <NR1>|MINimum|DEFault|MAXimum
        if (isAs3935Available)
        {
          int32_t noise = lightning.readNoiseLevel();
          if (checkInteger(isQuery, argc, argv, noiseMinimum, noiseDefault, noiseMaximum, noise, rc))
          {
            lightning.setNoiseLevel(noise);
          }
        }
        else
        {
          rc = -241;
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (isToken(subtoken, F("SPIKe")))
    { // :SENSe:SPIKe
      subtoken = nextToken();
      if (isToken(subtoken, F("REJection")))
      { // :SENSe:SPIKe:REJection? [MINimum|DEFault|MAXimum] & :SENSe:SPIKe:REJection <NR1>|MINimum|DEFault|MAXimum
        if (isAs3935Available)
        {
          int32_t spike = lightning.readSpikeRejection();
          if (checkInteger(isQuery, argc, argv, spikeMinimum, spikeDefault, spikeMaximum, spike, rc))
          {
            lightning.spikeRejection(spike);
          }
        }
        else
        {
          rc = -241;
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (isToken(subtoken, F("LIGHtning")))
    { // :SENSe:LIGHtning
      subtoken = nextToken();
      if (isThreshold(subtoken))
      { // :SENSe:LIGHtning:THReshold? [MINimum|DEFault|MAXimum] & :SENSe:LIGHtning:THReshold 1|5|9|16|MINimum|DEFault|MAXimum
        if (isAs3935Available)
        {
          int32_t threshold = lightning.readLightningThreshold();
          if (checkInteger(isQuery, argc, argv, lightningMinimum, lightningDefault, lightningMaximum, threshold, rc))
          {
            if ((threshold == 1) || (threshold == 5) || (threshold == 9) || (threshold == 16))
            {
              lightning.lightningThreshold(threshold);
            }
            else
            {
              rc = -224;
            }
          }
        }
        else
        {
          rc = -241; // Hardware missing
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (isToken(subtoken, F("WATChdog")))
    { // :SENSe:WATChdog
      subtoken = nextToken();
      if (isThreshold(subtoken))
      { // :SENSe:WATChdog:THReshold? [MINimum|DEFault|MAXimum] & :SENSe:WATChdog:THReshold <NR1>|MINimum|DEFault|MAXimum
        if (isAs3935Available)
        {
          int32_t watchdog = lightning.readWatchdogThreshold();
          if (checkInteger(isQuery, argc, argv, wdthresMinimum, wdthresDefault, wdthresMaximum, watchdog, rc))
          {
            lightning.watchdogThreshold(watchdog);
          }
        }
        else
        {
          rc = -241;
        }
      }
      else
      {
        rc = -113;
      }
    }
    else
    {
      rc = -113;
    }
  }
  else if (isToken(token, F("STATus")))
  { // :STATus
    subtoken = nextToken();
    if (isToken(subtoken, F("OPERation")))
    { // :STATus:OPERation
      subtoken = nextToken();
      parseScpiRegister(operReg, subtoken, isQuery, argc, argv, rc);
    }
    else if (!isQuery && isToken(subtoken, F("PRESet")))
    { // :STATus:PRESet
      rc = compareArgumentsCount(0, argc);
      if (rc == 0)
      {
        regPresetAll();
        updateStb();
      }
    }
    else if (isToken(subtoken, F("QUEStionable")))
    { // :STATus:QUEStionable
      subtoken = nextToken();
      if (isToken(subtoken, F("VOLTage")))
      { // :STATus:QUEStionable:VOLTage
        subtoken = nextToken();
        parseScpiRegister(voltReg, subtoken, isQuery, argc, argv, rc);
      }
      else if (isToken(subtoken, F("TIME")))
      { // :STATus:QUEStionable:TIME
        subtoken = nextToken();
        parseScpiRegister(timeReg, subtoken, isQuery, argc, argv, rc);
      }
      else if (isToken(subtoken, F("TEMPerature")))
      { // :STATus:QUEStionable:TEMPerature
        subtoken = nextToken();
        parseScpiRegister(tempReg, subtoken, isQuery, argc, argv, rc);
      }
      else if (isToken(subtoken, F("CALibration")))
      { // :STATus:QUEStionable:CALibration
        subtoken = nextToken();
        parseScpiRegister(caliReg, subtoken, isQuery, argc, argv, rc);
      }
      else if (isToken(subtoken, F("LIGHtning")))
      { // :STATus:QUEStionable:LIGHtning
        subtoken = nextToken();
        parseScpiRegister(lighReg, subtoken, isQuery, argc, argv, rc);
      }
      else
      {
        parseScpiRegister(quesReg, subtoken, isQuery, argc, argv, rc);
      }
    }
    else
    {
      rc = -113;
    }
  }
  else if (isToken(token, F("SYSTem")))
  { // :SYSTem
    subtoken = nextToken();
    if (isQuery && isToken(subtoken, F("ERRor")))
    { // :SYSTem:ERRor?
      subtoken = nextToken();
      bool codeOnly = false;
      if (isCount(subtoken))
      { // :SYSTem:ERRor:COUNt?
        displaySeparator();
        displayInteger(queueCount);
      }
      else
      {
        if (isToken(subtoken, F("CODE")))
        { // :SYSTem:ERRor:CODE?
          codeOnly = true;
          subtoken = nextToken();
        }
        if ((subtoken == NULL) || (isToken(subtoken, F("NEXT"))))
        { // :SYSTem:ERRor[:CODE][:NEXT]?
          int16_t errcode = popError();
          displaySeparator();
          if (codeOnly)
          {
            displayInteger(errcode);
          }
          else
          {
            displayError(errcode);
          }
          scpiOutput = true;
        }
        else if (isToken(subtoken, F("ALL")))
        { // :SYSTem:ERRor[:CODE]:ALL?
          displaySeparator();
          if (queueCount == 0)
          {
            if (codeOnly)
            {
              displayInteger(0);
            }
            else
            {
              displayError(0);
            }
          }
          else
          {
            bool comma = false;
            while (queueCount > 0)
            {
              if (comma) Serial.write(',');
              comma = true;
              int16_t errcode = popError();
              if (codeOnly)
              {
                displayInteger(errcode);
              }
              else
              {
                displayError(errcode);
              }
            }
          }
          scpiOutput = true;
        }
        else
        {
          rc = -113;
        }
      }
    }
    else if (isToken(subtoken, F("BEEPer")))
    { // :SYSTem:BEEPer
      subtoken = nextToken();
      if (!isQuery && ((subtoken == NULL) || isToken(subtoken, F("IMMediate"))))
      { // :SYSTem:BEEPer[:IMMediate] [<frequency>[,<time>]]
        int fr = buzzerFrequency;
        int du = buzzerDuration;
        if (argc > 0)
        { // <frequency>|MINimum|DEFault|MAXimum
          int32_t f;
          if (isMinimum(argv[0]))
          {
            f = buzzerFreqMinimum;
          }
          else if (isDefault(argv[0]))
          {
            f = buzzerFreqDefault;
          }
          else if (isMaximum(argv[0]))
          {
            f = buzzerFreqMaximum;
          }
          else
          {
            rc = isNR1(argv[0], f);
          }
          if (rc == 0)
          {
            if ((f < buzzerFreqMinimum) || (f > buzzerFreqMaximum))
            {
              rc = -222;
            }
            else
            {
              fr = f;
            }
          }
        }
        if (argc > 1)
        { // <time>|MINimum|DEFault|MAXimum
          float f;
          if (isMinimum(argv[1]))
          {
            f = buzzerDuraMinimum;
          }
          else if (isDefault(argv[1]))
          {
            f = buzzerDuraDefault;
          }
          else if (isMaximum(argv[1]))
          {
            f = buzzerDuraMaximum;
          }
          else
          {
            rc = isNRf(argv[1], f);
          }
          if (rc == 0)
          {
            if ((f < buzzerDuraMinimum) || (f > buzzerDuraMaximum))
            {
              rc = -222;
            }
            else
            {
              du = f * 1000.0;
            }
          }
        }
        if (rc == 0)
        {
          buzz(fr, du);
        }
      }
      else if (isToken(subtoken, F("FREQuency")))
      { // :SYSTem:BEEPer:FREQuency <NR1>|DEFault|MINimum|MAXImum
        int32_t frequency = buzzerFrequency;
        if (checkInteger(isQuery, argc, argv, buzzerFreqMinimum, buzzerFreqDefault, buzzerFreqMaximum, frequency, rc))
        {
          buzzerFrequency = frequency;
        }
      }
      else if (isState(subtoken))
      { // :SYSTem:BEEPer:STATe? [DEFault] et :SYSTem:BEEPer:STATe <boolean>|DEFault
        bool state = buzzerEnabled;
        if (checkBoolean(isQuery, argc, argv, buzzerStateDefault, state, rc))
        {
          buzzerEnabled = state;
        }
      }
      else if (isToken(subtoken, F("TIME")))
      { // :SYSTem:BEEPer:TIME
        float time = buzzerDuration / 1000.0;
        if (checkFloat(isQuery, argc, argv, buzzerDuraMinimum, buzzerDuraDefault, buzzerDuraMaximum, time, rc))
        {
          buzzerDuration = (int)(time * 1000.0);
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (isToken(subtoken, F("DISTurber")))
    { // :SYSTem:DISTurber
      subtoken = nextToken();      
      if (isState(subtoken))
      { // :SYSTem:DISTurber:STATe? [DEFault] & :SYSTem:DISTurber:STATe <boolean>|DEFault
        if (isAs3935Available)
        {
          bool state = lightning.readMaskDisturber() == 0;
          if (checkBoolean(isQuery, argc, argv, disturbDefault, state, rc))
          {
            isDisturberMasked = !state;
            lightning.maskDisturber(state?0:1);
          }
        }
        else
        {
          rc = -241; // Hardware missing
        }
      }
      else
      {
        rc = -113;
      }
    }
    else if (isToken(subtoken, F("TIME")))
    { // :SYSTem:TIME?
      if (isQuery)
      {
        RTCTime currentTime;
        displaySeparator();
        if (RTC.getTime(currentTime))
        {
          displayInteger(currentTime.getHour());
          Serial.write(',');
          displayInteger(currentTime.getMinutes());
          Serial.write(',');
          displayInteger(currentTime.getSeconds());
        }
        else
        {
          displayTime(0);
        }
        regBitWrite(timeReg, TIME_NTST, !RTC.isRunning());
      }
      else
      {
        // TODO
      }
    }
    else if (isToken(subtoken, F("DATE")))
    { // :SYSTem:DATE?
      if (isQuery)
      {
        uint16_t d = currentDateToWord();
        regBitWrite(timeReg, TIME_NTST, !RTC.isRunning());
        displaySeparator();
        displayDate(d);
      }
      else
      {
        // TODO
      }
    }
    else if (isQuery && isToken(subtoken, F("UPTime")))
    { // :SYSTem:UPTime?
      if (argc == 0)
      { // Affiche +j,+h,+m,+s depuis le démarrage de l'appareil (certains appareils affichent "hh:mm:ss")
        displaySeparator();
        if (isUptimeOvfl)
        {
          displayFloat(SCPI_NAN);
        }
        else
        { // 2^32 / 1000 = 4294967.295
          uint32_t sec = ((uint32_t)uptimeWraps * 4294967UL) + (now / 1000UL);
          displayInteger(sec / 86400UL);  // Up-time en jour
          Serial.write(',');
          sec %= 86400UL;
          displayInteger(sec / 3600UL);   // Up-time en heure
          Serial.write(',');
          sec %= 3600UL;
          displayInteger(sec / 60UL);     // Up-time en minute
          Serial.write(',');
          sec %= 60UL;
          displayInteger(sec);            // Up-time en seconde
        }
      }
      else
      {
        rc = -108;
      }
    }
    else if (isToken(subtoken, F("SERial")))
    { // :SYSTem:SERial? && :SYSTem:SERial
      if (isQuery)
      {
        displaySeparator();
        displaySerial();
      }
      else
      {
        rc = compareArgumentsCount(1, argc);
        uint8_t sn[8] = {0};
        if (rc == 0)
        {
          uint8_t i = 0;
          char *s = argv[0];
          char c;
          while (((c = s[i]) != '\0') && (rc == 0))
          {
            if (i > 15)
            {
              rc = -151; // Invalid string data
            }
            else if ((c >= '0') && (c <= '9'))
            {
              c = c - '0';
            }
            else if ((c >= 'A') && (c <= 'F'))
            {
              c = c - 'A' + 0x0A;
            }
            else if ((c >= 'a') && (c <= 'f'))
            {
              c = c - 'a' + 0x0A;
            }
            else
            {
              rc = -151; // Invalid string data
            }
            if (rc == 0)
            {
              if ((i % 2) == 0)
              {
                sn[i / 2] = c & 0x0F;
              }
              else
              {
                sn[i / 2] += c << 4;
              }
              i++;
            }
          }
          if ((rc == 0) && (i < 16))
          {
            rc = -151; // Invalid string data
          }
        }
        if (rc == 0)
        {
          if (isProtected)
          {
            rc = -203; // Command protected
          }
          else
          {
            for (uint8_t i=0; i<8; i++)
            {
              EEPROM.update(i, sn[i]);
              EEPROM.update(i+8, ~sn[i]);
            }
            bool eepromfail = false;
            for (uint8_t i=0; i<8; i++)
            {
              eepromfail |= EEPROM.read(i) != sn[i];
              eepromfail |= EEPROM.read(i+8) != ~sn[i];
            }
            if (eepromfail)
            {
              rc = -311; // Memory error
            }
          }
        }
      }
    }
    else if (isQuery && isToken(subtoken, F("VERSion")))
    { // :SYSTem:VERSion?
      if (argc == 0)
      {
        displaySeparator();
        Serial.print(F("1999.0")); // SCPI version
        scpiOutput = true;
      }
      else
      {
        rc = -108;
      }
    }
    else
    {
      rc = -113;
    }
  }
  else
  {
    rc = -113;
  }
  return rc;
}

/**
 * @brief Indique si un caractère est un espace au sens IEEE 488.2 (§7.4.1.2 : codes ASCII 1 à 32, sauf LF).
 * @param c Caractère à tester.
 * @return true si `c` est considéré comme un espace.
 */
bool isBlank(uint8_t c)
{ // IEEE 488.2 §7.4.1.2 définit le white space comme les codes ASCII 0–9 et 11–32 (soit tout caractère ≤ 32 sauf LF)
  return (c > 0) && (c <= 32) && (c != '\n');
}

/**
 * @brief Traite une ou plusieurs commandes/requêtes SCPI reçues sur le port série.
 * @param command Chaîne de commande à traiter (peut contenir une liste de commandes séparées par ';').
 */
void processSCPICommands(char *command)
{
  scpiOutput = false;
  if (command == nullptr) return;
#ifdef DEBUG_PARSING
  Serial.println();
  Serial.print(command);
  Serial.println("[");
#endif
  int rc = 0;

  // Analyse la commande SCPI (header)
  // Note: ne gère pas les suffixe
  uint8_t rootlen = 0;
  size_t i = 0;
  bool isNotEol = (command[i] != '\0');
  size_t header = 0, roottre = 0;
  while ((rc == 0) && isNotEol) // ligne vide autorisée (sans erreur)
  {
    // supprime les espaces éventuels avant l'en-tête
    while (isBlank(command[i]))
    {
      i++;
    }

    // Cherche l'entête et recopie éventuellement la racine de l'en-tête précédente en cas de chemin relatif
    char c = command[i];
    if (c == ';')
    { // Commande vide interdite
      rc = -102; // Syntax error
    }
    else if (c == '\0')
    { // Ligne vide autorisée (sans erreur)
      isNotEol = false;
    }
    else if ((c == '*') || (c == ':') || ((c >= 'A') && (c <= 'Z')) || ((c >= 'a') && (c <= 'z')))
    { // Debut de commande ou question
      // Identifie le début de l'en-tête
      bool isStandard = false;
      bool isQuery = false;
      bool inHeader = true;
      bool noArguments = false;
      // Serial.print("i: ");
      // Serial.println(i);
      switch (c)
      {
        case ':':
          // Défini la racine du nouveau chemin absolu
          i++;
          rootlen = 0;
          roottre = i;
          header = i;
          break;
        case '*':
          // Commande standard. N'a aucun impact sur le chemin
          isStandard = true;
          i++;
          header = i;
          break;
        default:
          // Recopie la racine de l'en-tête précédente en cas de chemin relatif
          // Serial.print("root: ");
          // Serial.print(roottre);
          // Serial.print(" (");
          // Serial.print(rootlen);
          // Serial.println(")");
          if (rootlen == 0)
          {
            roottre = i;
          }
          else if (rootlen > i)
          {
            rc = -310; // System error
          }
          else
          {
            size_t j = rootlen + roottre;
            for (int8_t idx = rootlen; idx > 0; idx--)
            {
              if ((i == 0) || (j == 0))
              {
                rc = -310; // System error
              }
              else
              {
                i--;
                j--;
                command[i] = command[j]?command[j]:':'; // remplace les '\0' générés par strtok() par des ':'
#ifdef DEBUG_PARSING
                Serial.write(command[j]?command[j]:':');
#endif
              }
            }
            roottre = i;
            header = i;
            i += rootlen;
#ifdef DEBUG_PARSING
            Serial.write('~');
            Serial.println(header);
#endif
          }
          break;
      }
      // ici l'entête complète commence à l'index "header" et le chemin racine commence à l'index "roottre" avec 
      // une longueur de "rootlen". Si ce n'est pas une commande standard, "header" = "roottre".
      // Si c'est une commande standard, "isStandard" est vrai.

      // Cherche la fin de l'en-tête, en agrandissant éventuellement le chemin racine
      uint8_t mnemonicLen = 0;
      while (inHeader && isNotEol && (rc == 0))
      {
        c = command[i];
        if (c == '\0')
        {
          isNotEol = false;
          inHeader = false;
        }
        else if (!isQuery && (c >= 'a') && (c <= 'z'))
        { // S'assurer que le premier caractère est une lettre
          command[i] = toupper(c);
          mnemonicLen++;
        }
        else if (!isQuery && (c >= 'A') && (c <= 'Z'))
        {
          mnemonicLen++;
        }
        else if (!isQuery && (c == ':'))
        {
          if (isStandard)
          {
            rc = -101; // Invalid character
          } 
          else if (mnemonicLen == 0)
          { // "::" interdit
            rc = -102; // Syntax error
          }
          else if (mnemonicLen > 12)
          {
            rc = -112; // Program mnemonic too long
          }
          else
          {
            rootlen = 1 + i - roottre;
          }
          mnemonicLen = 0;
        }
        else if (!isQuery && ((c == '_') || ((c >= '0') && (c <= '9'))))
        {
          if (mnemonicLen == 0)
          { // Le premier caractère doit être une lettre
            rc = -101; // Invalid character
          }
          mnemonicLen++;
        }
        else if (c == '?')
        {
          if (mnemonicLen == 0)
          { // Le premier caractère doit être une lettre
            rc = -101; // Invalid character
          }
          else if (mnemonicLen > 12)
          {
            rc = -112; // Program mnemonic too long
          }
          else if (isQuery)
          {
            rc = -101; // Invalid character
          }
          else
          {
            isQuery = true;
            command[i] = '\0';
          }
        }
        else if (isBlank(c) || (c == ';'))
        { // TODO: Corriger B16
          command[i] = '\0';
          inHeader = false;
          noArguments = (c == ';');
        }
        else
        {
          rc = -101;
        }

        if ((rc == 0) && isNotEol)
        {
          i++;
          c = command[i];
        }
      }

#ifdef DEBUG_PARSING
      Serial.print("Header: '");
      Serial.print(&command[header]);
      Serial.print("' (");
      if (rc == 0)
      {
        if (isStandard) Serial.print("standard ");
        Serial.print(isQuery?"query":"command");
      }
      else
      {
        Serial.print("ERROR=");
        Serial.print(rc);
      }
      Serial.println(")");
#endif

      int argc = 0;
      char *argv[8];
      if (noArguments || !isNotEol)
      { // On sait qu'il n'y a pas de paramètres 
        argc = 0;
      }
      else
      {
        bool inArg = false;
        bool onlyBlank = false;
        size_t arg = 0;
        // Recherche des paramètres éventuels
        while (isNotEol && (c != ';') && (rc == 0))
        {
          // supprime les espaces éventuels en début de paramètre
          if (!inArg)
          {
            while (isBlank(command[i]))
            {
              i++;
            }
          }
          c = command[i];
          if ((c == '\0') || (c == ',') || (c == ';'))
          {
            if (inArg)
            {
              argv[argc-1] = &command[arg];
              command[i] = '\0';
              inArg = false;
#ifdef DEBUG_PARSING
              Serial.print(" - argument #");
              Serial.print(argc);
              Serial.print(": '");
              Serial.print(&command[arg]);
              Serial.println("'");
#endif
            }
            else if ((c == ',') || (argc > 0))
            { // Paramètre vide
              rc = -102;
            }
            isNotEol = (c != '\0');
            onlyBlank = false;
          }
          else if ((c & 0x80) != 0)
          { // uniquement ASCII 7-bit autorisé
            rc = -101;
          }
          else if (c == ':')
          {
            rc = -103;
          }
          else
          {
            if (!inArg)
            { // Début paramètre
              if (argc >= SCPI_MAX_ARGS)
              {
                rc = -108;
              }
              else
              {
                arg = i;
                argc++;
                inArg = true;
                if ((c == '"') || (c == '\''))
                { // Paramètre type chaîne de caractère
                  i++;
                  while((command[i] != c) && (command[i] != '\0'))
                  {
                    i++;
                  }
                  if (command[i] == '\0')
                  { // Chaîne non fermée
                    rc = -102;
                    isNotEol = false;
                  }
                  else
                  {
                    onlyBlank = true;
                  }
                }
              }
            }
            else if (onlyBlank && !isBlank(c))
            {
              rc = -102;
            }
          }

          if ((rc == 0) && isNotEol)
          {
            i++;
          }
        }
      }
#ifndef DEBUG_PARSING
      if (rc == 0)
      {
        // Serial.println (" --> OK to execute");
        rc = isStandard?processSCPIStandardCommand(&command[header], isQuery, argc, argv)
                       :processSCPISpecificCommand(&command[header], isQuery, argc, argv);
      }
#else
      if (!isNotEol)
      {
        Serial.println("**EOL");
      }
#endif
    }
    else
    { // Caractère interdit en début de commande
      rc = -110;
    }
  }
  if (rc != 0)
  {
    pushError(rc);
#ifdef DEBUG_PARSING
    while (i>0)
    {
      i--;
      Serial.write(' ');
    }
    Serial.println("^");
    displayError(rc);
    Serial.println();
#endif
  }
  if (scpiOutput)
  {
    Serial.println();
    scpiOutput = false;
  }
}

/**
 * @brief Traite la détection effective d'un éclair : incrémente le compteur, active brièvement la LED et le
 * buzzer (durée et fréquence proportionnelles à l'énergie et à la distance), et masque temporairement les
 * parasites générés par le PWM de la LED.
 */
void lightningDetected()
{
  if (lightningCount < UINT32_MAX)
  {
    lightningCount++;
  }
  else if (!isLightningCountOvfl)
  {
    isLightningCountOvfl = true;
    pushError(-222); // Data out of range
  }
  regBitSet(lighReg, LIGH_LIGH); // Éclair détecté
  digitalWrite(lightningLed, HIGH);
  delay(2);
  digitalWrite(lightningLed, LOW);
  regBitClear(lighReg, LIGH_LIGH); // Fin éclair 

  regBitSet(lighReg, LIGH_STOR); // Orage en cours

  uint8_t distance = constrain(lightning_distance, DISTANCE_MIN, DISTANCE_MAX);
  uint32_t energy = constrain(lightning_energy, ENERGIE_MIN, ENERGIE_MAX);
  // L'énergie de l'AS335 augmente très vite, on utilise la racine carrée (ou sqrt/log)
  // pour éviter que seuls les très gros impacts ne dépassent la durée minimale.
  float energieCompress = sqrt((float)energy);
  float maxCompress = sqrt((float)ENERGIE_MAX);
  uint16_t duree = map((long)energieCompress, 0, (long)maxCompress, DUREE_MIN, DUREE_MAX);
  uint16_t freq = map(distance, DISTANCE_MIN, DISTANCE_MAX, FREQ_MAX, FREQ_MIN);

  lightning.maskDisturber(true); // Le PWM de la LED perturbe le capteur (génère des disturbers)
  // if (buzzerEnabled) tone(passiveBuzzer, freq, duree);
  if (buzzerEnabled) buzz(freq, duree);
  energyLed = map((long)energieCompress, 0, (long)maxCompress, 0, 255);
  analogWrite(lightningLed, energyLed);
}

/**
 * @brief Fonction d'initialisation Arduino : configure les broches, l'état SCPI de base, recharge la
 * configuration persistante et la calibration depuis l'EEPROM, puis initialise le capteur AS3935 et son auto-calibration.
 */
/***************************************************************************************/
void setup()
{
  // 1. E/S
  // When lightning is detected the interrupt pin goes HIGH.
  pinMode(lightningInt, INPUT);
  pinMode(lightningLed, OUTPUT);
  pinMode(noiseLed, OUTPUT);
  pinMode(disturbLed, OUTPUT);
  pinMode(passiveBuzzer, OUTPUT);
  pinMode(errorLed, OUTPUT);

  Serial.begin(115200);
  SPI.begin();

  // 2. État SCPI de base
  scpiBuffer[0] = '\0';
  scpiBufferIndex = 0;
  scpiError = 0;
  quesReg.con = 0;
  operReg.con = 0;
  voltReg.con = 0;
  timeReg.con = 0;
  tempReg.con = 0;
  caliReg.con = 0;
  lighReg.con = 0;
#ifdef SCPI_MAP
  quesReg.map = scpiQuesMap;
  operReg.map = scpiOperMap;
  voltReg.map = nullptr;
  timeReg.map = nullptr;
  tempReg.map = nullptr;
  caliReg.map = nullptr;
  lighReg.map = nullptr;
#endif  
  regPresetAll();
  regClearEvents();
  clearErrorQueue();
  scpiEse = 0x00;
  scpiSre = 0x00;
  scpiEsr = 0x80; // PON
  pushError(-500);

  // 3. Réglages de l'appareil
#ifndef FIRMWARE_VERSION
  formatBuildDate();
#endif
  uptimeWraps = 0;
  isUptimeOvfl = false;
  resetSettings();
  resetStatistics(STAT_ALL);
  temperature = SCPI_NAN;
  voltVcc = UINT32_MAX;

  // 4. Configuration persistante (peut écraser ESE/SRE si *PSC 0)
#ifdef SCPI_PSC
  int16_t rc = loadPowerOnConfiguration();
  if (rc != 0) pushError(rc);
#endif
  rc = loadCalibration();
  if (rc != 0) pushError(rc);

  // 3. Matériel (capteur, etc.)
  regBitWrite(timeReg, TIME_NTST, !RTC.isRunning()); // Time not set
  regBitWrite(timeReg, TIME_NTAV, !RTC.begin()); // RTC not available
  regBitSet(timeReg, TIME_BATL); // RTC battery low
  regBitSet(timeReg, TIME_EXNA); // External time reference not available

  // 6. Journalisation
  updateStb();

  isAs3935Available = lightning.begin(spiCS);
  if (!isAs3935Available)
  { // TODO: en SPI, le begin renvoi toujours true...
    pushError(-241);
  }
  else if (bitRead(caliReg.con, CALI_CDCO))
  { // Appliquer la calibration
    uint8_t cap = cal.capafreq[0] >> 4;
    lightning.tuneCap(cap);
  }
  uint8_t calibrationResult;
  rc = selfCalibration(calibrationResult);
/*
  // When the distance to the storm is estimated, it takes into account other
  // lightning that was sensed in the past 15 minutes. If you want to reset
  // time, then you can call this function. 

  //lightning.clearStatistics();

  // The power down function has a BIG "gotcha". When you wake up the board
  // after power down, the internal oscillators will be recalibrated. They are
  // recalibrated according to the resonance frequency of the antenna - which
  // should be around 500kHz. It's highly recommended that you calibrate your
  // antenna before using these two functions, or you run the risk of schewing
  // the timing of the chip. 

  //lightning.powerDown(); 
  //delay(1000);
  //if( lightning.wakeUp() ) 
   // Serial.println("Successfully woken up!");  
  //else 
    //Serial.println("Error recalibrating internal osciallator on wake up."); 
  
*/
  lastLedUpdate = 0;
}

/**
 * @brief Boucle principale Arduino : gère l'horodatage (uptime), l'extinction du buzzer, la réception et le
 * traitement des commandes SCPI sur le port série, l'extinction progressive de la LED d'énergie, ainsi que
 * les interruptions du capteur AS3935 (bruit, parasite, éclair, changement de distance) ou l'affichage de la
 * source de calibration manuelle en cours.
 */
void loop()
{
  static uint32_t lastMillis = 0;
  now = millis();

  // Gère l'uptime en seconde avec détection du débordement pour tenir environ 8 ans :)
  if (now < lastMillis)
  {
    if (uptimeWraps <= UINT8_MAX)
    {
      uptimeWraps++;
    }
    else if (!isUptimeOvfl)
    {
      isUptimeOvfl = true;
      pushError(-222); // Data out of range
    }
  }

  // Arrête le buzzer si la durée est écoulée
  if (buzzActive && ((int32_t)(now - buzzUntil) >= 0))
  {
#ifdef BUZZER_OC2B
    TCCR2A = TCCR2B = 0;
#else
    noTone(passiveBuzzer);
#endif
    buzzActive = false;
  }

  if (Serial.available())
  {
    uint8_t c = Serial.read();
    static uint8_t oldc = '\0';
    if (c == 0x98) // SPE : Serial Poll (j'ai mis le bit 7 à 1 pour différencier du flux normal)
    {
      updateStb();
      Serial.write(0x18); // SPE Serial Poll Enable
      Serial.write(scpiStb); // Revoi directement le registre de statut
      Serial.write(0x19); // SPD Serial Poll Disable
      Serial.flush();
    }
    else if (c == '\n')
    {
      if ((scpiBufferIndex > 0) && (oldc == '\r'))
      {
        scpiBufferIndex--;
      }
      scpiBuffer[scpiBufferIndex] = '\0';
#ifdef DEBUG_PARSING
      Serial.print("INPUT: [");
      Serial.print(scpiBuffer);
      Serial.print("] SCPI Error=");
      Serial.println(scpiError);
#endif
      if (scpiError == 0)
      {
        processSCPICommands(scpiBuffer);
      }
      else
      {
        pushError(scpiError);
        scpiError = 0;
      }
      scpiBufferIndex = 0;
      oldc = '\0';
    }
    else if ((!isBlank(c)) || ((scpiBufferIndex > 0) && (oldc != ' ')))
    {
      if (scpiBufferIndex < (SCPI_BUFFER_SIZE - 1))
      {
        if (isblank(c)) c = ' ';
        scpiBuffer[scpiBufferIndex++] = c;
        oldc = c;
      }
      else
      {
        scpiError = -112;
      }
    }
  }

  if ((energyLed > 0) && ((now - lastLedUpdate) > 100))
  {
    lastLedUpdate = now;
    energyLed--;
    if (energyLed == 0)
    {
      digitalWrite(lightningLed, LOW);
      lightning.maskDisturber(isDisturberMasked);
    }
    else
    {
      analogWrite(lightningLed, energyLed);
    }
  }
  
  switch (sourceCal)
  {
    case CAL_SOUR_NONE: // Fonctionnement normal (pas en calibration)
      if (digitalRead(lightningInt) == HIGH)
      { // Whenever an interrupt is issued, the external unit should wait 2ms before reading the Interrupt register
        // because the AS3935 need time to write values to registers.
        delay(3);
        // Hardware has alerted us to an event, now we read the interrupt register
        // to see exactly what it is.
        // After a lightning strike: values must be read before 1 second
        // After a disturber: values must be read before 1,5 second
        uint8_t iregValue = lightning.readInterruptReg();
        switch (iregValue)
        {
          case TOO_NOISY_INT: // Noise too high
            lightning_intReason = iregValue;
            if (noiseCount < UINT32_MAX)
            {
              noiseCount++;
            }
            else if (!isNoiseCountOvfl)
            {
              isNoiseCountOvfl = true;
              pushError(-222); // Data out of range
            }
            regBitSet(lighReg, LIGH_NOIS); // Bruit détecté
            digitalWrite(noiseLed, HIGH);
            delay(2);
            digitalWrite(noiseLed, LOW);
            regBitClear(lighReg, LIGH_NOIS); // Fin bruit
            break;
          case DISTURBER_INT: // Disturber detected !
            lightning_intReason = iregValue;
            if (disturberCount < UINT32_MAX)
            {
              disturberCount++;
            }
            else if (!isDisturberCountOvfl)
            {
              isDisturberCountOvfl = true;
              pushError(-222); // Data out of range
            }
            regBitSet(lighReg, LIGH_DIST); // Parasite détecté
            digitalWrite(disturbLed, HIGH);
            delay(2);
            digitalWrite(disturbLed, LOW);
            regBitClear(lighReg, LIGH_DIST); // Fin parasite
            break;
          case LIGHTNING_INT: // Lightning strike detected !
            lightning_intReason = iregValue;
            readDistance(true);
            readEnergy(true);
            lightningDetected();
            lastLedUpdate = now;
            break;
          case 0: // Distance changed
            digitalWrite(noiseLed, HIGH);
            digitalWrite(lightningLed, HIGH);
            readDistance(false);
            updateStormTrend();
            digitalWrite(noiseLed, LOW);
            digitalWrite(lightningLed, LOW);
            break;
          case 2:  // Données des registres invalides (lues trop tôt ou trop tard) ou problème de calibration
            digitalWrite(noiseLed, HIGH);
            digitalWrite(disturbLed, HIGH);
            digitalWrite(lightningLed, HIGH);
            delay(5);
            // Serial.write('%');
            // Serial.println(iregValue);
            digitalWrite(noiseLed, LOW);
            digitalWrite(disturbLed, LOW);
            digitalWrite(lightningLed, LOW);
            break;
          default:
            if (isAs3935Available)
            {
              pushError(-240);
              isAs3935Available = false;
            }
            break;
        }
      }
      break;
    case CAL_SOUR_TRCO:
      digitalWrite(noiseLed, HIGH);
      digitalWrite(disturbLed, LOW);
      digitalWrite(lightningLed, LOW);
      break;
    case CAL_SOUR_SRCO:
      digitalWrite(noiseLed, LOW);
      digitalWrite(disturbLed, HIGH);
      digitalWrite(lightningLed, LOW);
      break;
    case CAL_SOUR_LCO:
      digitalWrite(noiseLed, LOW);
      digitalWrite(disturbLed, LOW);
      digitalWrite(lightningLed, HIGH);
      break;
  }
}
