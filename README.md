# Lightning Detector

Application WPF C# pour lire des données depuis un port série COM et les visualiser dans une interface Windows simple.

## Structure

- src/LightningDetector.App : application principale WPF
  - Models : modèles de données
  - Services : gestion du port série
  - ViewModels : logique d'interface

## Prérequis

- .NET 8 SDK
- Windows (WPF)

## Démarrage

```powershell
dotnet build lightning_detector.sln
```

## Format des données reçues sur le port série

Le port série respecte le protocole SCPI (IEEE 488.2). Il n'y a donc rien d'affiché à la connexion.
Chaque commande est envoyée sur 1 ligne. Celles qui contiennent un point d'interrogation (`?`) retournent une réponse, sur une ligne également. Ne pas attendre plus de 2 secondes une réponse et indiquer un "timeout" si rien n'a été retourné au bout de ce temps. Les commandes qui ne contiennent pas de point d'interrogation ne retournent rien, pas besoin d'attendre quoi que ce soit.
Plusieurs commandes peuvent être mise à la suite sur une même ligne, dans ce cas, elles sont séparées par un point-virgule `;`. S'il y a plusieurs commandes nécessitants une réponse envoyées en même temps, les réponse seront données dans l'ordre, séparées par un point-vurgule aussi.
Par exemple, si la commande suivante est envoyée:

```txt
*IDN?;*CLS;*TST?
```

`*IDN?` et `*TST?` envoyent une réponse, mais pas `*CLS`. On pourrait donc par exemple avoir la réponse suivante:

```txt
ValTronix,LightningDetector,0,2026-08-06T02:27:24;0
```

Pour vérifier qu'on est bien connecté au capteur, il suffit d'envoyer la commande `*IDN?` et d'attendre maximum 500ms la réponse. On peut faire 8 tentatives de suite en cas de timeout.
La réponse est alors quelque chose qui ressemble à ça:

```txt
ValTronix,LightningDetector,0,2026-08-06T02:27:24
```

Il s'agit de 4 chaînes de caractères séparées par une virgule (`,`). Les 4 champs correspondent respectivement à :

1. Nom de la marque (doit être "ValTronix", sans tenir compte de la casse)
2. Le nom du capteur (doit être "LightningDetector", sans tenir compte de la casse)
3. Le numéro de série (peut être quelconque)
4. La version du firmware (qui est pour l'instant la date/heure de compilation)

Ensuite, il faut lancer un auto-test du capteur en envoyant la commande `*TST?`. Cette commande renvoi une ligne ne contenant que "0" si le test est OK (passage en vert de la pastille), ou autre chose en cas d'erreur. Ne pas faire plusieurs essai ici.

Puis il faut récupérer la configuration qui sera affiché de façon claire dans la zone "Configuration du capteur". À chaque modification d'un des paramètres, la commande correspondante est envoyée et la valeur est de nouveau relue pour vérifier qu'elle a bien été acceptée.

Pour simuler le *Serial Poll* du bus GPIB, une astuce a été faite. Il suffit d'envoyer l'octet `0x98`, l'Arduino va alors répondre par 3 octets `0x18` `STB` `0x19`. Le deuxième octet correspond au contenu du registre STB. Il est alors affiché sous forme de LEDs où chaque LED représente un bit. Attention: na pasenvoyer cet octet pendant qu'une commande est déjà en cours pour ne pas mélanger les flux.

Seuls les échanges manuels sur le port série (envois des commandes et réponses depuis la ligne "commande manuelle" ou les boutons de raccourcis en dessous) seront affichés dans la zone "Données brutes".

### Configuration du capteur

**Registre `AFE_GB` (`REG0x00[5:1]`) du capteur AC3935:** gain de l'étage d'entrée (AFE) configuré pour l'intérieur ou l'extérieur à afficher sous forme d'une drop-box avec ces 2 choix uniquement "intérieur" ou "extérieur". On récupère l'état actuel avec la commande `:SENSe:AFE?` qui retourne `IND` pour "intérieur" (`INDoor`) ou `OUT` pour l'extérieur (`OUTdoor`). Pour modifier la valeur, il suffit d'envoyer `:SENSe:AFE INDoor` ou `:SENSe:AFE OUTdoor`.

> Note: d'après la norme SCPI, que ce soit en entrée ou en sortie, les mots-clés ne sont pas sensibles à la casse. Dans la définition de chaque mot-clé (*header*) on peut voir des lettres en majuscules (exemple: `INDoor`) et éventuellement d'autres en minuscules. Cela permet de spécifier la **forme courte** du mot qui n'est alors composé de l'ensemble des lettres majuscules (ici `IND`) et la **forme longue** qui se compose de **toutes** les lettres (ici `INDOOR`). Autant la casse n'a pas d'importance (`INDOOR` = `indoor` = `InDoOr`, etc.), autant seules les 2 formes, courtes et longues, sont acceptées en entrée. Ainsi `IN` ou `outd` sont ne correspondent pas à ce mot-clé. La norme veut aussi qu'en sortie ce soit la forme courte en majuscule qui soit envoyée.

**Registre `NF_LEV` (`REG0x01[6:4]`) du capteur AC3935:** seuil de bruit se récupère via la commande `:SENSe:NOISe:THReshold?`. Les valeurs possibles sont les entiers entre `0` et `7` inclus. Pour définir la valeur, il suffit d'envoyer `:SENSe:NOISe:THReshold 4` par exemple pour la définir à `4`.

**Registre `WDTH` (`REG0x01[3:0]`) du capteur AC3935:** seuil de rejet des parasites. `:SENSe:DISTurber:REJect?` permet de récupérer le seuil de rejet des parasites. Pour changer le niveau, il suffit d'envoyer la même commande, sans le point d'intérogation mais suivi de la nouvelle valeur. Les valeurs possibles sont tous les entiers entre `0` et `11`. Exemple: `:SENSe:DISTurber:REJect 2` pour définir la valeur à `1`.

**Registre `MIN_NUM_LIGH` (`REG0x02[5:4]`) du capteur AS3935:** nombre minimum d'éclairs dans les 17 dernières minutes avant de lever une interruption. La commande `:SENSe:LIGHtning:THReshold?` permet de récupérer la valeur. et `:SENSe:LIGHtning:THReshold 1` de la définir à 1. Ce paramètre n'accepte que les 4 valeurs suivantes: `1`, `5`, `9` ou `16`.

**Registre `SREJ` (`REG0x02[3:0]`) du capteur AS3935:** rejet des pics. La commande `:SENSe:LIGHning:SPIKes?`.

**Registre `MASK_DIST` (`REG0x03[5]`) du capteur AS3935:** permet de ne pas déclencher d'interruption pour les parasites. On regarde l'état via la commande `:SENSe:DISTurber:STATe?` et on la modifie via la commande `:SENSe:DISTurber:STATe 1` pour la mettre à vrai (et donc ne pas déclencher d'interruption). Cela sera affiché sous forme d'une case à cocher pour indiquer si les perturbateurs sont masqués (`0` = non, `1` = oui).

### Interrogation régulière

Régulièrement, envoyer la commande ":SYST:UPT?;:CALCulate:LIGH?;DIST?;NOIS?" qui renvoi une ligne contenant 4 valeurs entières séparées par un point-virgule (';'). Exemple:

```txt
221;0;1775;47
```

1. "221" correspond à la réponse de la commande ":SYST:UPT?" qui indique l'up-time en seconde.
2. "0" correpond à la réponse à la commande ":CALCulate:LIGH?" qui indique le nombre d'éclairs captés depuis la mise en route (ou la remise à 0 des compteurs)
3. "1775" correpond à la réponse à la commande ":CALCulate:DIST?" qui indique le nombre de perturbateurs captés depuis la mise en route (ou la remise à 0 des compteurs)
4. "47"  au nombre de dépassement du seuil de bruit.

En ce qui concerne la fréquence pour exécuter la commande, il faudrait ajouter 2 valeurs différentes selon si un éclair a été détecté ou non mais par défaut je partirais sur une période de 10 secondes entre chaque test. Puis si le compteur du nombre d'éclair augmente, (la 2ème colonne), passer au 2ème intervalle, bien plus fréquent, disons chaque seconde jusqu'à la fin de l'orage (quand il n'y a plus de changement du nombre d'éclair pendant 15 minutes).

Si au moins un éclair est détecté (le compteur correspondant a été incrémenté), il faut envoyer la commande ":FETCh:ENERgy?;DISTance?" qui récupère le niveau d'énergie et la distance (en mètres) séparés par un point-virgule. Si une valeur est inconnue, elle sera remplacée par "NAN" (Not A Number). Si la distance est hors de portée, c'est la valeur "INF" qui sera renvoyée, ce qui correspond à "+infini". Exemple: "NAN;INF". Chaque éclair détecté sera ajouté au tableau de l'onglet "Vue synthétique" avec la date/heure de la détection.

### Envoie de commandes à la main

Il faut ajouter une ligne, juste sous la zone "Données brutes" permettant d'envoyer des commandes arbitraires. Si la ligne n'est pas vide, elle sera envoyée lors de l'appuis sur la touche Entrée. Il faut pouvoir récupérer les commandes précédentes via les flèches haut/bas. Pour référence, la liste de commandes et des codes d'erreurs figurent dans la suite de la documentation.

## Commandes SCPI

### Commandes standard

Les noms **en gras** indiquent les commandes IEEE qui doivent être obligatoirement implémentées.

| Mnémonique | Nom | Description |
| --- | --- | --- |
| `*CAL?` | Calibration Query | Démarre la calibration et retourne le résultat |
| `*CLS` | **Clear Status Command** | Efface tous les registres d'événement et la liste d'erreurs |
| `*ESE <NRf>` | **Standard Event Status Enable Command** | Défini le registre d'activation des événements |
| `*ESE?` | **Standard Event Status Enable Query** | Lit le registre d'activation des événements |
| `*ESR?` | **Standard Event Status Register Query** | Lit le registre des événements et le remet à 0 |
| `*IDN?` | **Identification Query** | Retourne le fabricant, le modèle, numéro de série, révision |
| `*OPC` | **Operation Complete Command** | Met à 1 le bit 0 du registre ESR |
| `*OPC?` | **Operation Complete Query** | Retourne 1 car toutes les tâches sont terminées (vu qu'on ne peut pas lancer de tâches de fond) |
| `*RCL <NRf>` | Recall Command | Recharge les réglages utilisateur |
| `*RST` | **Reset Command** | Recharge les réglages d'usine |
| `*SAV <NRf>` | Save Command | Sauvegarde les réglages utilisateur |
| `*SRE <NRf>` | **Service Request Enable Command** | Défini le registre de demande de service |
| `*SRE?` | **Service Request Enable Query** | Lit le registre de demande de service |
| `*STB?` | **Read Status Byte Query** | Lit le registre d'état |
| `*TST?` | **Self-Test Query** | Exécute un auto-test et renvoi le résultat |
| `*WAI` | **Wait-to-Continue Command** | Attend jusqu'à ce que toutes le commandes précédentes soient terminées |

### Commandes spécifiques

```txt
:SENSe:AFE[:GAIN] INDoor|OUTdoor|DEFault
:SENSe:AFE[:GAIN]? [DEFault]
:SENSe:NOISe:THReshold <NR1>|MINimum|DEFault|MAXimum
:SENSe:NOISe:THReshold? [MINimum|DEFault|MAXimum]
:SENSe:WATChdog:THReshold <NR1>|MINimum|DEFault|MAXimum
:SENSe:WATChdog:THReshold? [MINimum|DEFault|MAXimum]
:SENSe:LIGHtning:THReshold 1|5|9|16|DEFault
:SENSe:LIGHtning:THReshold? [DEFault]
:SENSe:SPIKe:REJection <NR1>|MINimum|DEFault|MAXimum
:SENSe:SPIKe:REJection? [MINimum|DEFault|MAXimum]
:SYSTem:DISTurber:STATe <boolean>|DEFault
:SYSTem:DISTurber:STATe? [DEFault]

:SYSTem:STATistics:CLEar

:INITiate // TODO

:FETCh:DISTance?
:FETCh:ENERgy?
:FETCh:TYPE[:CODE]?

:MEASure:DISTance?
:MEASure:ENERgy[:PERCent]?
:MEASure:ALL?
:MEASure:VOLT[:DC]?

:CALibrate // TODO

:DIAGnose:REGister? <NR1>
:DIAGnose:REGister <NR1>,<NR1>

:CALCulate:LIGHtning[:COUNt]?
:CALCulate:LIGHtning:CLEar
:CALCulate:LIGHtning:ENERgy[:PERCent]:MINimum?
:CALCulate:LIGHtning:ENERgy[:PERCent]:MAXimum?
:CALCulate:LIGHtning:ENERgy[:PERCent]:AVERage?
:CALCulate:LIGHtning:DISTance:MINimum?
:CALCulate:LIGHtning:DISTance:MAXimum?
:CALCulate:LIGHtning:DISTance:AVERage?
:CALCulate:DISTurber[:COUNt]?
:CALCulate:DISTurber:CLEar
:CALCulate:ENERgy[:COUNt]?
:CALCulate:ENERgy:CLEAr
:CALCulate:ENERgy[:PERCent]:MINimum?
:CALCulate:ENERgy[:PERCent]:MAXimum?
:CALCulate:ENERgy[:PERCent]:AVERage?
:CALCulate:NOISe[:COUNt]?
:CALCulate:NOISe:CLEar
:CALCulate:CLEar

:STATus:OPERation:CONDition?
:STATus:OPERation:ENABle <NR1>
:STATus:OPERation:ENABle?
:STATus:OPERation[:EVENt]?
:STATus:OPERation:MAP <NR1>,<NR1>
:STATus:OPERation:MAP? [<NR1>]
:STATus:OPERation:NTRansition <NR1>
:STATus:OPERation:NTRansition?
:STATus:OPERation:PTRansition <NR1>
:STATus:OPERation:PTRansition?
:STATus:PRESet
:STATus:QUEStionable:CONDition?
:STATus:QUEStionable:ENABle <NR1>
:STATus:QUEStionable:ENABle?
:STATus:QUEStionable[:EVENt]?
:STATus:QUEStionable:MAP <NR1>,<NR1>
:STATus:QUEStionable:MAP? [<NR1>]
:STATus:QUEStionable:NTRansition <NR1>
:STATus:QUEStionable:NTRansition?
:STATus:QUEStionable:PTRansition <NR1>
:STATus:QUEStionable:PTRansition?

:SYSTem:ERRor:COUNt?
:SYSTem:ERRor:CODE[:NEXT]?
:SYSTem:ERRor:CODE:ALL?
:SYSTem:ERRor[:NEXT]?
:SYSTem:ERRor:ALL?

:SYSTem:BEEPer[:IMMediate]
:SYSTem:BEEPer:FREQuency <NRf>|MINimum|DEFault|MAXimum
:SYSTem:BEEPer:FREQuency? [MINimum|DEFault|MAXimum]
:SYSTem:BEEPer:STATe? [DEFault]
:SYSTem:BEEPer:STATe <boolean>|DEFault
:SYSTem:BEEPer:TIME <NRf>|MINimum|DEFault|MAXimum
:SYSTem:BEEPer:TIME? [MINimum|DEFault|MAXimum]

:SYSTem:UPTime?
:SYSTem:VERSion?
```

### Mot de passe de calibration

Les réglages de calibration sont protégés en écriture par un mot de passe, vérifié par
`:CALibration:SECure:STATe`. **Le mot de passe par défaut (celui chargé en usine ou après une
perte de la calibration en EEPROM) est `VTX1234`.**

Comme ce mot de passe par défaut est visible dans le code source (public sur ce dépôt), il ne
protège en pratique qu'un accès physique/série non protégé par ailleurs — il est recommandé de le
changer dès la mise en service de l'appareil :

1. Déverrouiller avec le mot de passe actuel :
   `:CALibration:SECure:STATe FALSE,VTX1234`
2. Définir un nouveau mot de passe (chaîne non guillemetée de 12 caractères maximum, doit
   commencer par une lettre A-Z, peut contenir lettres/chiffres/underscore) :
   `:CALibration:SECure:CODE <nouveau_mot_de_passe>`
3. Reverrouiller (optionnel, un cycle d'alimentation reverrouille aussi automatiquement) :
   `:CALibration:SECure:STATe TRUE`

Le nouveau mot de passe est sauvegardé immédiatement en EEPROM et survit aux redémarrages.
