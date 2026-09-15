# Rapport d'analyse — `Lightning_Detector.ino`

**Cible** : Arduino Nano (ATmega328P, 32 ko Flash / 2 ko RAM / 1 ko EEPROM) + AS3935 (SPI)
**Fichier analysé** : `Lightning_Detector.ino` — 4052 lignes
**Documents de référence** : `Détecteur_d_orage.pdf` (notes de conception), IEEE 488.2, SCPI-99
**Date** : 14 août 2026
**Nature** : analyse statique par lecture. Aucun fichier fourni n'a été modifié.

---

## 1. Synthèse exécutive

Le firmware est ambitieux et globalement bien structuré : les 13 commandes obligatoires IEEE 488.2 sont présentes, la file d'erreurs respecte la règle du `-350` en écrasement de la dernière entrée, le mécanisme de chemin relatif dans les messages composés est implémenté (rare dans un firmware de cette taille), et la refactorisation en `struct ScpiRegister` est la bonne direction.

En revanche, l'analyse a identifié **17 bugs bloquants ou majeurs**, dont plusieurs invalident silencieusement le comportement attendu :

| # | Résumé | Gravité |
|---|--------|---------|
| B01 | `updateQues()` mappe TEMPerature sur le bit **3** au lieu du bit **4** | 🔴 Bloquant |
| B02 | `*CLS` efface `*ESE`/`*SRE` (interdit) et n'efface **pas** l'ESR (obligatoire) | 🔴 Bloquant |
| B03 | Traces de debug `" >> "` émises par `isNRf()` dans le flux de réponse | 🔴 Bloquant |
| B04 | `checkByte()` : affectation de la valeur **malgré** une erreur de parsing | 🔴 Bloquant |
| B05 | `clearOperQues()` : variable `i` masquée (shadowing) → mapping erroné | 🔴 Bloquant |
| B06 | `(rc = 0)` au lieu de `(rc == 0)` dans `:DIAGnostic:LIGHtning` | 🔴 Bloquant |
| B07 | `rc = 111` au lieu de `rc = -111` (2 occurrences) | 🔴 Bloquant |
| B08 | `:SYSTem:BEEPer` : `argv[0]` testé au lieu de `argv[1]` + `tone()` non gardé | 🔴 Bloquant |
| B09 | `isDisturberMasked` reçoit la valeur **inversée** | 🟠 Majeur |
| B10 | `:STATus` sans `else` final → header inconnu accepté silencieusement | 🟠 Majeur |
| B11 | `VOLT` au lieu de `VOLTage` → forme longue rejetée (2 endroits) | 🟠 Majeur |
| B12 | Bit MAV (STB.4) toujours à 1 | 🟠 Majeur |
| B13 | `setup()` : `Serial.begin()` après les premiers usages de `Serial` ; ordre d'init incohérent | 🟠 Majeur |
| B14 | Dépassement possible de `argv[8]` (débordement de pile) | 🟠 Majeur |
| B15 | `loop()` : lecture de `scpiBuffer[scpiBufferIndex]` (case non écrite) | 🟠 Majeur |
| B16 | Message composé `HEADER<espace>;` → `-102` injustifié | 🟠 Majeur |
| B17 | `:FETCh:ENERgy:PERCent?` retourne une fraction 0–1 affichée sur 2 décimales | 🟠 Majeur |

La cascade `:STAT:QUES:LIGH → QUES.10 → STB.3` est **conceptuellement correcte** mais souffre de trois défauts : le conflit avec `:STATus:...:MAP`, l'absence de `updateStb()` après lecture d'un `:EVENt?`, et une récursion mutuelle non gardée (§7).

Côté empreinte Flash, les gisements identifiés totalisent **6 à 8 ko** (sur 30 ko utilisables), dont ~3 ko pour la seule élimination du virgule flottante et de `snprintf`, avant même la factorisation du parseur (§8).

---

## 2. Périmètre et méthode

**Analysé** : lecture ligne à ligne du `.ino`, vérification croisée avec les notes PDF, simulation mentale du parseur sur une vingtaine de messages types, traçage des chemins de la cascade de registres.

**Non analysé** — voir §11 pour le détail : la bibliothèque `SparkFun_AS3935` (non fournie), la compilation réelle (pas de toolchain ni de réseau dans l'environnement), le comportement sur matériel.

**Convention** : les numéros de ligne renvoient au fichier fourni tel quel.

---

## 3. Bugs bloquants

### B01 — `updateQues()` : QUEStionable:TEMPerature mappé sur le mauvais bit

**Ligne 317**

```cpp
regBitWrite(quesReg, 0, regCascade(voltReg));
regBitWrite(quesReg, 3, regCascade(tempReg));   // ← bit 3 = POWer
regBitWrite(quesReg, 8, regCascade(caliReg));
regBitWrite(quesReg, 10, regCascade(lighReg));
```

SCPI-99 définit le registre QUEStionable ainsi : bit 0 VOLTage, 1 CURRent, 2 TIME, **3 POWer**, **4 TEMPerature**, 5 FREQuency, 6 PHASe, 7 MODulation, 8 CALibration. Votre propre schéma Mermaid dans le PDF indique bien `Q4["4: TEMPerature"]`. Un contrôleur qui masque `:STAT:QUES:ENAB 16` pour surveiller la température ne verra jamais rien remonter.

**Correctif** : `regBitWrite(quesReg, 4, regCascade(tempReg));`

Je recommande fortement de remplacer les littéraux par des `enum` :

```cpp
enum : uint8_t {
  QUES_VOLT = 0, QUES_CURR = 1, QUES_TIME = 2, QUES_POW  = 3,
  QUES_TEMP = 4, QUES_FREQ = 5, QUES_PHAS = 6, QUES_MOD  = 7,
  QUES_CAL  = 8, QUES_WARN = 9, QUES_LIGH = 10,
  QUES_INST = 13, QUES_CMDW = 14
};
```

---

### B02 — `*CLS` viole IEEE 488.2 sur deux points

**Lignes 514–525 (`regClearEvents`) et 2070–2081 (`*CLS`)**

```cpp
void regClearEvents()
{
  operReg.eve = 0x0000;
  quesReg.eve = 0x0000;
  voltReg.eve = 0x0000;
  tempReg.eve = 0x0000;
  caliReg.eve = 0x0000;
  lighReg.eve = 0x0000;
  scpiEse = 0x00;     // ← INTERDIT
  scpiSre = 0x00;     // ← INTERDIT
  updateStb();
}
```

IEEE 488.2 §10.3 est explicite : `*CLS` efface **tous les registres d'événements** et la file d'erreurs, mais **ne modifie aucun registre d'activation** (ESE, SRE, `:STATus:...:ENABle`). Ici, c'est exactement l'inverse qui se produit :

- `scpiEsr` — le Standard Event Status Register, dont l'effacement est la raison d'être principale de `*CLS` — **n'est jamais remis à zéro** ;
- `scpiEse` et `scpiSre`, qui doivent être préservés, sont écrasés.

Conséquence pratique : un contrôleur qui fait `*ESE 60; *SRE 32` puis `*CLS` perd toute sa configuration de service request, et le bit ESB de STB reste collé à sa valeur précédente.

**Correctif** :

```cpp
void regClearEvents()
{
  operReg.eve = quesReg.eve = voltReg.eve = 0;
  tempReg.eve = caliReg.eve = lighReg.eve = 0;
  scpiEsr = 0x00;              // ← à ajouter
  updateStb();
}
```

Et supprimer les deux lignes `scpiEse = 0; scpiSre = 0;`. Attention : cela impacte `setup()` (voir B13) où `regClearEvents()` détruisait justement la configuration rechargée par `loadPowerOnConfiguration()`.

⚠️ Effet de bord : `*CLS` effacera désormais aussi le bit PON (ESR bit 7), ce qui est le comportement normatif attendu.

---

### B03 — Traces de debug dans le flux de réponse SCPI

**Lignes 1584–1585, dans `isNRf()`**

```cpp
v = v + deci;
if (mantisNeg) v = -v;
if (exponentNeg) exponent = -exponent;
Serial.print(" >> ");        // ← trace de debug oubliée
Serial.println(v);           // ← idem
```

Toute commande prenant un paramètre `<NRf>` (`:SYST:BEEP:TIME 0.5`, `:SYST:BEEP 1000,0.3`) injecte ` >> 0.50` + CRLF au milieu du flux. Un contrôleur SCPI qui lit ligne par ligne se désynchronise immédiatement.

**Correctif** : supprimer les deux lignes, ou les encadrer par `#ifdef DEBUG_PARSING`.

Trois autres émissions non sollicitées sont à traiter dans le même esprit :

| Ligne | Émission | Problème |
|---|---|---|
| 4011–4012 | `Serial.write('%'); Serial.println(iregValue);` | Sortie asynchrone hors protocole requête/réponse |
| 3344–3346 | bloc `DEBUG_PARSING` | OK (conditionnel) |

Le cas de la ligne 4011 est traité en B18 : la valeur `0` du registre d'interruption est un cas **légitime** de l'AS3935.

---

### B04 — `checkByte()` : affectation malgré l'erreur de parsing

**Lignes 1693–1707**

```cpp
rc = compareArgumentsCount(1, ac);
if (rc == 0)
{
  int32_t v = -1;
  rc = isNR1(av[0], v);
  if ((rc == 0) && ((v < min) || (v > max)))
  {
    rc = -222;
  }
  else                    // ← atteint aussi quand isNR1() a échoué
  {
    value = v;            // ← v vaut -1 → 255 après troncature uint8_t
    return true;          // ← "succès" alors que rc != 0
  }
}
```

Si `isNR1()` échoue (`rc = -121`), la condition composée est fausse et le `else` s'exécute quand même. Résultat pour `*ESE ABC` : `scpiEse` prend la valeur 255, `updateStb()` est appelé, `savePowerOnConfiguration()` écrit en EEPROM, et le code d'erreur retourné à l'appelant est ensuite écrasé.

Pour `*RCL XYZ` c'est pire : `recallState(255)` est appelé et retourne `-310` (System error) qui remplace le vrai `-121` (Invalid character in number).

**Correctif** :

```cpp
rc = isNR1(av[0], v);
if (rc == 0)
{
  if ((v < min) || (v > max)) { rc = -222; }
  else { value = v; return true; }
}
```

Le **même motif** est présent trois fois dans `:SYSTem:BEEPer` (lignes 3196–3203 et 3224–3231) — voir B08. Une relecture ciblée de tous les `if ((rc == 0) && …) … else …` est recommandée : c'est un anti-pattern systématique dans ce fichier.

---

### B05 — `clearOperQues()` : masquage de variable (shadowing)

**Lignes 479–511**

```cpp
for (uint8_t i = 0; i < 15; i++)          // ← i externe
{
  if (scpiOperMap[i] != 0)
  {
    bool found = false;
    uint8_t j = queueTail;
    for (uint8_t i = 0; i < queueCount; i++)     // ← i interne masque l'externe
    {
      if (scpiOperMap[i] == errorQueue[j]) found = true;   // ← utilise le i INTERNE
      j++;
      j %= ERROR_QUEUE_SIZE;
    }
    if (!found)
    {
      regBitClear(operReg, i);              // ← i externe (correct ici)
    }
  }
  …même schéma pour scpiQuesMap…
}
```

La comparaison porte sur `scpiOperMap[i_interne]` — c'est-à-dire sur une entrée de mapping arbitraire indexée par la position dans la file d'erreurs — au lieu de `scpiOperMap[i_externe]`, le mapping du bit en cours d'examen. La fonction efface donc des bits qui devraient rester actifs, et en conserve d'autres qui devraient tomber.

Le bug est doublement masqué : `queueCount ≤ 8` alors que la boucle externe va jusqu'à 15, donc les bits 8 à 14 sont systématiquement comparés à des entrées de mapping hors du sous-ensemble balayé.

**Correctif** : renommer la variable interne (`k`) et corriger l'indexation.

```cpp
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
  // idem pour scpiQuesMap[bit] / quesReg
}
```

> **Note générale** : compiler avec `-Wshadow -Wall -Wextra` aurait détecté ce bug, ainsi que B06, B07 et les variables non initialisées signalées plus loin. Dans l'IDE Arduino : *Préférences → Avertissements du compilateur → Tous*. C'est, de loin, l'action au meilleur rapport effort/bénéfice de tout ce rapport.

---

### B06 — Affectation au lieu de comparaison

**Ligne 2555, dans `:DIAGnostic:LIGHtning`**

```cpp
rc = isNR1(argv[0], dist);
if ((rc = 0) && ((dist < 1) || (dist > 40)))   // ← rc = 0, pas rc == 0
{
  rc = -222;
}
```

`(rc = 0)` affecte 0 à `rc` (effaçant une éventuelle erreur de parsing) puis s'évalue à *faux*, ce qui court-circuite tout le test de plage. Résultat : `:DIAG:LIGH 999` ou `:DIAG:LIGH abc` sont acceptés sans erreur.

**Correctif** : `if ((rc == 0) && …)`.

Deux problèmes supplémentaires dans le même bloc (lignes 2560–2563) :

```cpp
readDistance(true);          // ← lit le capteur ET pollue les statistiques
lightning_distance = dist;   // ← puis écrase la valeur lue
readEnergy(true);
lightningDetected();
```

- ces quatre appels s'exécutent **même si `rc != 0`** ;
- `readDistance(true)` met à jour min/max/total avec la distance *réelle* du capteur, puis la valeur simulée écrase `lightning_distance` : les statistiques et la valeur affichée divergent.

**Correctif** :

```cpp
if (rc == 0)
{
  lightning_distance = dist;
  readEnergy(true);
  lightningDetected();
}
```

Cette commande n'apparaît pas dans la liste du PDF. Puisqu'elle sert manifestement à l'injection de test, je suggère de la documenter explicitement comme telle et de la protéger (voir §10.4).

---

### B07 — Code d'erreur positif (typo de signe)

**Lignes 3460 et 3472**

```cpp
else if ((c >= 'a') && (c <= 'z'))
{
  if (numbersOnly)
  {
    rc = 111;      // ← devrait être -111
  }
  …
```

`111` est un code positif, donc réservé aux erreurs spécifiques appareil. Trois conséquences :

1. `pushError(111)` ne passe pas le filtre `(error <= -100) && (error >= -499)` → **aucun bit ESR n'est armé** ;
2. `displayError(111)` affiche `+111,""` (chaîne vide, aucun `case` correspondant) ;
3. l'erreur réelle (« Header suffix out of range » / caractère invalide dans l'en-tête) est perdue.

**Correctif** : `rc = -111;` aux deux endroits. Le code `-111` correspond à *Header separator error* ; pour un caractère alphabétique après un suffixe numérique, `-114` (*Header suffix out of range*) ou `-101` (*Invalid character*) sont sans doute plus exacts — à trancher selon la sémantique que vous retenez pour les suffixes (§9.7).

---

### B08 — `:SYSTem:BEEPer[:IMMediate]` : mauvais argument et absence de garde

**Lignes 3205–3234**

```cpp
if (argc > 1)
{ // <time>|MINimum|DEFault|MAXimum
  float f;
  if (isMinimum(argv[0]))        // ← argv[0] = la FRÉQUENCE, pas la durée
  {
    f = buzzerDuraMinimum;
  }
  else if (isDefault(argv[0]))   // ← idem
  …
  else
  {
    rc = isNRf(argv[1], f);      // ← ici c'est bien argv[1]
  }
  if ((rc == 0) && ((f < buzzerDuraMinimum) || (f > buzzerDuraMaximum)))
  {
    rc = -222;
  }
  else
  {
    du = f * 1000.0;             // ← f non initialisé si isNRf a échoué (B04 bis)
  }
}
tone(passiveBuzzer, fr, du);     // ← exécuté même si rc != 0
delay(du);                       // ← blocage jusqu'à 5 s
```

Quatre défauts cumulés :

1. **`argv[0]` au lieu de `argv[1]`** pour les mots-clés MIN/DEF/MAX de la durée. `:SYST:BEEP 2000,MAX` teste `isMinimum("2000")`, `isDefault("2000")`, `isMaximum("2000")` → tous faux → tombe sur `isNRf(argv[1], f)` avec `argv[1] = "MAX"` → `-121`. Et `:SYST:BEEP MAX,0.5` applique la durée maximale au lieu de la fréquence maximale.
2. **`float f;` non initialisé** puis utilisé dans la branche `else` en cas d'erreur (même motif que B04). Idem pour `int32_t f;` ligne 3179 dans le bloc fréquence.
3. **`tone()`/`delay()` hors de toute garde `rc == 0`** : un bip retentit même sur commande invalide.
4. **`delay(du)` bloque jusqu'à 5 s** : à 115200 bauds, le tampon série matériel de 64 octets déborde en ~5,5 ms de trafic continu. Toute commande envoyée pendant le bip est perdue **silencieusement** (pas d'erreur `-363`).

**Correctif** (structure suggérée) :

```cpp
int32_t fr = buzzerFrequency;
float   du = buzzerDuration / 1000.0;
if ((rc == 0) && (argc > 0)) rc = parseScaled(argv[0], buzzerFreqMinimum,
                                              buzzerFreqDefault, buzzerFreqMaximum, fr);
if ((rc == 0) && (argc > 1)) rc = parseScaledF(argv[1], buzzerDuraMinimum,
                                               buzzerDuraDefault, buzzerDuraMaximum, du);
if ((rc == 0) && (argc > 2)) rc = -108;
if (rc == 0) tone(passiveBuzzer, fr, (unsigned long)(du * 1000.0));  // sans delay()
```

Le `delay()` est inutile : `tone()` avec durée est déjà non bloquant sur AVR. C'est un gain immédiat de réactivité. Le seul `delay()` réellement nécessaire est celui de `systemBeep()` si vous voulez enchaîner des bips — remplacez-le par une machine à états dans `loop()` (§10.1).

---

## 4. Bugs majeurs

### B09 — `isDisturberMasked` reçoit la valeur inversée

**Lignes 3272–3277**

```cpp
bool state = lightning.readMaskDisturber() == 0;   // state = "parasites RAPPORTÉS"
if (checkBoolean(isQuery, argc, argv, disturbDefault, state, rc))
{
  isDisturberMasked = state;                 // ← INVERSION : state = non-masqué
  lightning.maskDisturber(state?0:1);        // ← correct
}
```

`state` signifie « les parasites sont rapportés » (donc *non* masqués), mais il est affecté tel quel à `isDisturberMasked`. La variable est utilisée ligne 3948, dans `loop()`, pour restaurer le masquage après l'extinction de la LED :

```cpp
lightning.maskDisturber(isDisturberMasked);   // restaure l'inverse du réglage utilisateur
```

Séquence de reproduction : `:SYST:DIST:STAT OFF` (masquer) → le capteur est correctement masqué → premier éclair détecté → fondu de LED terminé → `maskDisturber(false)` → le masquage utilisateur est **silencieusement annulé**.

**Correctif** : `isDisturberMasked = !state;`

**Question de sémantique associée** : `disturbDefault = false` (ligne 53). Si `STATe` signifie « rapport des parasites activé », alors la valeur par défaut du composant (`MASK_DIST = 0`, parasites rapportés) correspond à `STATe ON`, donc `disturbDefault` devrait valoir `true`. Actuellement `:SYST:DIST:STAT DEF` masque les parasites, ce qui ne reproduit pas l'état d'usine du composant ni celui obtenu par `*RST`. Je recommande `const bool disturbDefault = true;` et un commentaire explicite sur la convention retenue.

---

### B10 — Branche `:STATus` sans `else` final

**Lignes 3050–3095**

```cpp
else if (isToken(token, F("STATus")))
{
  subtoken = nextToken();
  if (isToken(subtoken, F("OPERation"))) { … }
  else if (!isQuery && isToken(subtoken, F("PRESet"))) { … }
  else if (isToken(subtoken, F("QUEStionable"))) { … }
  // ← pas de else { rc = -113; }
}
```

`:STAT:FOO?`, `:STAT?`, `:STAT:PRES?` (query sur un event command) retournent `rc = 0` : aucune sortie, aucune erreur. Un contrôleur qui attend une réponse reste bloqué en timeout sans jamais savoir pourquoi.

**Correctif** : ajouter `else { rc = -113; }`.

C'est le seul sous-arbre de premier niveau où le `else` manque — `SENSe`, `DIAGnostic`, `FETCh`, `MEASure`, `CALCulate`, `SYSTem` le possèdent tous. Vérifiez également le cas `:STAT:PRES?` : il faut `-113` (l'en-tête interrogatif n'existe pas) plutôt que d'ignorer le `?`.

---

### B11 — Mnémonique `VOLT` : forme longue impossible

**Lignes 2663 (`:MEASure:VOLT`) et 3070 (`:STATus:QUEStionable:VOLT`)**

```cpp
else if (isToken(subtoken, F("VOLT")))
```

Dans `isToken()`, la convention est : les majuscules du littéral forment la **forme courte**, l'ensemble forme la **forme longue**. Avec `F("VOLT")`, les deux formes sont identiques : `VOLT`. Or SCPI définit le mnémonique `VOLTage`, et un contrôleur a le droit d'envoyer la forme longue.

Conséquences :
- `:MEAS:VOLTAGE?` → `-113`
- `:STAT:QUES:VOLTAGE:COND?` → tombe dans le `else` générique et est traité comme un sous-nœud de QUEStionable → `-113`

Le PDF lui-même utilise `:STATus:QUEStionable:VOLTage:CONDition?`. Votre implémentation ne répond donc pas à votre propre documentation.

**Correctif** : `F("VOLTage")` aux deux endroits.

J'ai vérifié tous les autres littéraux passés à `isToken()` : `CODE`, `GAIN`, `MAP`, `ALL`, `TYPE`, `TIME`, `NAN`, `UP`, `DOWN`, `ON`, `OFF` et les commandes communes n'ont pas de forme longue distincte — ils sont corrects. `VOLT` est le seul cas problématique.

---

### B12 — Bit MAV (STB bit 4) toujours à 1

**Ligne 336**

```cpp
bitWrite(scpiStb, 4, Serial.availableForWrite() < SERIAL_TX_BUFFER_SIZE);
```

Sur AVR, `HardwareSerial::availableForWrite()` retourne au maximum `SERIAL_TX_BUFFER_SIZE - 1` (une case est réservée pour distinguer plein de vide dans le tampon circulaire). La condition `< SERIAL_TX_BUFFER_SIZE` est donc **toujours vraie**, tampon vide compris. MAV vaut 1 en permanence.

Un contrôleur qui fait du serial poll pour savoir s'il peut lire une réponse lira systématiquement 1 et tentera une lecture qui partira en timeout.

**Correctif minimal** : `< (SERIAL_TX_BUFFER_SIZE - 1)`.

**Correctif conceptuel** (recommandé) : MAV signifie « le *Output Queue* contient un message », pas « le tampon UART matériel n'est pas vide ». Dans votre architecture, la réponse est émise immédiatement et de façon synchrone dans `processSCPICommands()`, donc l'Output Queue est vide dès le retour de la fonction. La formulation correcte est un drapeau :

```cpp
volatile bool mavPending = false;
// dans processSCPICommands(), à la fin :
if (scpiOutput) { Serial.println(); mavPending = true; scpiOutput = false; }
// après Serial.flush() ou en tête de la lecture série suivante :
mavPending = (Serial.availableForWrite() < (SERIAL_TX_BUFFER_SIZE - 1));
```

Notez que sur une liaison série simple (sans handshake IEEE-488 réel), MAV a une utilité limitée ; l'important est qu'il ne soit pas *faux*.

---

### B13 — `setup()` : ordre d'initialisation

**Lignes 3726–3763**

```cpp
formatBuildDate();
…
scpiEsr = 0x80;                             // PON
int16_t rc = loadPowerOnConfiguration();    // recharge ESE/SRE/ENABle
regPresetAll();
regClearEvents();                           // ← ÉCRASE scpiEse et scpiSre (B02)
clearErrorQueue();
resetSettings();                            // ← teste isAs3935Available (encore false)
resetStatistics(0xff);
pushError(-500);                            // ← systemBeep() + updateStb() → Serial
if (rc != 0) pushError(rc);

Serial.begin(115200);                       // ← APRÈS les premiers usages de Serial
SPI.begin();
isAs3935Available = lightning.beginSPI(spiCS);   // ← APRÈS resetSettings()
```

Quatre problèmes distincts :

1. **`Serial.begin()` trop tard.** `updateStb()` (appelé depuis `pushError`) interroge `Serial.availableForWrite()` sur un objet non initialisé. Sur AVR cela ne plante pas (les indices statiques valent 0) mais c'est un comportement non défini formellement, et cela empêchera tout diagnostic si vous ajoutez un jour une trace.

2. **`regClearEvents()` annule `loadPowerOnConfiguration()`.** L'objet de `*PSC 0` est précisément de restaurer ESE/SRE au démarrage ; deux lignes plus loin ils sont remis à zéro. La fonctionnalité `*PSC` est donc entièrement inopérante. Ce point disparaît une fois B02 corrigé, mais l'ordre doit quand même être : reset → chargement.

3. **`resetSettings()` avant l'init du capteur.** `isAs3935Available` vaut `false` (initialisation statique) au moment de l'appel, donc `lightning.resetSettings()` n'est jamais exécuté : l'AS3935 démarre avec ses valeurs de reset matériel plutôt qu'avec vos valeurs par défaut. C'est fonctionnellement acceptable (elles coïncident) mais c'est fragile et non intentionnel.

4. **`pushError(-500)` déclenche `systemBeep()`** alors que `buzzerEnabled`/`buzzerFrequency` viennent juste d'être initialisés — OK ici, mais la boucle de bips de fin de `setup()` (lignes 3882–3886) rejoue un bip par erreur en attente, ce qui fait au minimum 2 bips au démarrage (PON + éventuel `-241`). Comportement volontaire ? À documenter.

**Ordre recommandé** :

```cpp
void setup()
{
  // 1. E/S
  pinMode(...);
  Serial.begin(115200);
  SPI.begin();

  // 2. Matériel capteur
  isAs3935Available = probeAs3935();      // voir B19

  // 3. État SCPI de base
  regPresetAll();
  clearErrorQueue();
  scpiEsr = 0x80;                          // PON

  // 4. Réglages appareil
  resetSettings();                         // maintenant isAs3935Available est valide
  resetStatistics(RESET_ALL);

  // 5. Configuration persistante (peut écraser ESE/SRE si *PSC 0)
  int16_t rc = loadPowerOnConfiguration();

  // 6. Journalisation
  pushError(-500);
  if (rc != 0) pushError(rc);
  updateStb();
}
```

---

### B14 — Débordement possible du tableau `argv[8]`

**Lignes 3377 et 3610–3614**

```cpp
char *argv[8];
…
if (!inArg)
{
  arg = i;
  argc++;               // ← aucune borne
  inArg = true;
  …
}
…
argv[argc-1] = &command[arg];   // ← argc peut valoir 9, 10, …
```

`argc` n'est jamais borné. Avec 9 paramètres ou plus, `argv[8]`, `argv[9]`… écrivent au-delà du tableau, qui est une **variable locale de `processSCPICommands()`, donc sur la pile**. On écrase l'adresse de retour ou les variables voisines (`i`, `header`, `roottre`, `rc`).

Le tampon d'entrée fait 64 octets, donc 9 paramètres tiennent facilement : `:SYST:BEEP 1,2,3,4,5,6,7,8,9` fait 29 caractères. **C'est exploitable trivialement et provoque un reset ou un comportement erratique.**

**Correctif** :

```cpp
#define SCPI_MAX_ARGS 8
…
if (!inArg)
{
  if (argc >= SCPI_MAX_ARGS) { rc = -108; break; }   // Parameter not allowed
  arg = i;
  argc++;
  …
}
```

Le code d'erreur `-108` (*Parameter not allowed*) est adapté ; certains constructeurs utilisent `-223` (*Too much data*). Comme aucune commande du firmware n'accepte plus de 2 paramètres, vous pouvez même réduire `SCPI_MAX_ARGS` à 4 et récupérer 8 octets de pile.

---

### B15 — `loop()` : lecture d'une case non écrite du tampon

**Ligne 3922**

```cpp
else if (((c != ' ') && (c != '\t')) ||
         ((scpiBufferIndex > 0) && (scpiBuffer[scpiBufferIndex] != ' ')))
```

`scpiBufferIndex` désigne la **prochaine case libre**. Pour tester le caractère précédemment stocké, il faut `scpiBuffer[scpiBufferIndex - 1]`. Actuellement on lit un octet résiduel de la ligne précédente (le tampon n'est pas ré-effacé entre deux messages, seul `scpiBufferIndex` est remis à 0).

La logique de compactage des espaces est donc non déterministe : elle dépend du contenu du message précédent. `:SYST:BEEP  1000` peut passer ou non selon l'historique.

**Correctif** :

```cpp
else
{
  if (c == '\t') c = ' ';
  bool skip = (c == ' ') && ((scpiBufferIndex == 0) ||
                             (scpiBuffer[scpiBufferIndex - 1] == ' '));
  if (!skip)
  {
    if (scpiBufferIndex < (SCPI_BUFFER_SIZE - 1)) scpiBuffer[scpiBufferIndex++] = c;
    else scpiError = -363;   // Input buffer overrun (voir §6.5)
  }
}
```

⚠️ **Attention** : ce compactage s'applique aussi **à l'intérieur des chaînes de caractères entre guillemets**. `:CAL:PROT:STAT OFF,"mot  de  passe"` verrait ses espaces doublés réduits. Comme le PDF prévoit `:CALibration:PROTected:STATe OFF,"1234"` et un champ `Note` de 128 bits dans les données de calibration, il faudra à terme désactiver le compactage entre guillemets — donc le déplacer du niveau `loop()` (flux d'octets) vers le parseur (qui, lui, connaît le contexte). C'est le bon endroit de toute façon.

---

### B16 — `HEADER<espace>;` produit une erreur `-102` injustifiée

**Lignes 3510–3528 et 3362–3365**

Traçons `*CLS ;*RST` :

1. La boucle d'en-tête rencontre l'espace : `command[i] = '\0'`, `inHeader = false`, `c == ' '` donc ni `isQuery` ni `argc = -1`.
2. En bas de boucle : `i++`, `c = command[i]` → `c = ';'`.
3. On sort de la boucle d'en-tête avec `argc == 0` et `isNotEol == true` → on entre dans la boucle de paramètres.
4. Condition de la boucle : `while (isNotEol && (c != ';') && (rc == 0))` → `c == ';'` → **la boucle ne s'exécute pas et `i` n'est jamais incrémenté**.
5. Retour à la boucle externe : `command[i]` vaut toujours `';'` → `rc = -102` (« Commande vide interdite »).

Sans espace, `*CLS;*RST` fonctionne (le `case ';'` positionne `argc = -1` et `i` avance). Le comportement dépend donc d'un espace optionnel, ce qui est contraire à la règle SCPI selon laquelle un séparateur d'espaces est autorisé partout après l'en-tête.

**Correctif** : dans la branche de fin d'en-tête, traiter `;` de la même façon quel que soit le chemin, par exemple en consommant le `;` avant de sortir :

```cpp
else if ((c == '?') || isBlank(c) || (c == ';'))
{
  command[i] = '\0';
  inHeader = false;
  if (c == '?') { isQuery = true; i++; c = command[i]; if (c == '\0') isNotEol = false; }
  while (isBlank(c)) { i++; c = command[i]; }     // ← consommer les espaces
  if (c == ';') { argc = -1; }
  else if (c == '\0') { isNotEol = false; }
}
```

Puis, après le traitement de la commande, avancer d'un cran si `command[i] == ';'`. À vérifier soigneusement sur les cas de test du §12.

---

### B17 — `PERCent` retourne une fraction, affichée sur 2 décimales

**Lignes 1946, 2761, 2784, 2808, 2928, 2948, 2969**

```cpp
Serial.print((double)lightning_energy / (double)ENERGIE_MAX);
```

Deux problèmes cumulés :

1. **Ce n'est pas un pourcentage** : la valeur est comprise entre 0 et 1. Le mnémonique `PERCent` implique 0–100. Il manque un `* 100.0`.
2. **`Serial.print(double)` affiche 2 décimales par défaut.** L'énergie de l'AS3935 est fortement concentrée dans le bas de l'échelle 21 bits : pour une énergie typique de quelques milliers d'unités, la fraction vaut ~0,002 et s'affiche **`0.00`**. La quasi-totalité des mesures renvoie donc `0.00`.

Corrigé en pourcentage, on obtient 0,2 % → affiché `0.20`, ce qui reste très pauvre.

**Correctif recommandé** : abandonner le flottant (voir aussi §8.1, gain Flash majeur) et exprimer le pourcentage en **centièmes de pour-cent** sous forme entière, ou fixer explicitement la précision :

```cpp
// Option A — entier, échelle ppm (0..1000000), aucun flottant
uint32_t ppm = (uint32_t)(((uint64_t)energy * 1000000UL) / ENERGIE_MAX);

// Option B — conserver le flottant mais fixer la précision
Serial.print(100.0 * energy / ENERGIE_MAX, 4);   // "0.1907"
```

Le PDF précise que le format ASCII par défaut est une mantisse à 6 chiffres, au-delà de laquelle on bascule en notation scientifique. `Serial.print()` ne sait pas faire cela ; il faudra de toute façon écrire un formateur dédié (§8.1) si vous voulez respecter `:FORMat[:DATA] ASCii,<n>`.

---

## 5. Bugs mineurs et points de robustesse

### B18 — Interruption `INT = 0` non gérée (purge des statistiques)

**Lignes 4006–4016**

```cpp
default:
  digitalWrite(noiseLed, HIGH);
  …
  Serial.write('%');
  Serial.println(iregValue);   // ← sortie non sollicitée
```

D'après la datasheet — et vos propres notes PDF : *« Si l'AS3935 génère une interruption et que le registre d'interruption REG0x03[3:0] = 0000, cela signifie que l'estimation de distance a changé en raison de la purge d'événements anciens »*. C'est un événement **normal et significatif** : c'est précisément lui qui permet de détecter qu'un orage s'éloigne.

Actuellement il tombe dans le `default`, allume les trois LED et **émet `%0` sur la liaison série**, ce qui corrompt le protocole.

**Correctif** :

```cpp
case 0x00:   // Purge des statistiques → l'estimation de distance a changé
  readDistance(false);
  updateStormTrend();          // voir §10.2 (bits 1 et 2 de QUES:LIGH)
  break;
default:
  pushError(-240);             // Hardware error, sans sortie non sollicitée
  break;
```

### B19 — `isAs3935Available` n'est jamais faux

**Ligne 3759**, commentaire du code : *« en SPI, le begin renvoi toujours true… »*

Toutes les gardes `if (isAs3935Available) … else rc = -241;` sont donc du code mort, et `*TST?` (bit 1) ne détecte jamais l'absence du capteur.

**Correctif** : sonder un registre à valeur connue. Après reset, `REG0x00 = 0x24` (AFE_GB = b10010, PWD = 0) et `REG0x01 = 0x22` (NF_LEV = b010, WDTH = b0010). Un test de plausibilité robuste :

```cpp
bool probeAs3935()
{
  writeRegister(0x08, 0x0F);                 // TUN_CAP = 15
  bool ok = ((readRegister(0x08) & 0x0F) == 0x0F);
  writeRegister(0x08, 0x00);
  ok &= ((readRegister(0x08) & 0x0F) == 0x00);
  return ok;
}
```

Un test aller-retour en écriture/lecture est plus fiable qu'une comparaison à une valeur de reset, car il valide aussi les lignes MOSI/MISO/SCK.

### B20 — `readVcc()` : division par zéro possible et seuils incohérents

**Lignes 1128–1137**

```cpp
result = 1125300L / result;                       // ← si result == 0 → UB
regBitWrite(voltReg, 8, (result <= 2400));        // sous-tension : 2,4 V
regBitWrite(voltReg, 0, (result >= 5500));        // sur-tension  : 5,5 V
…
bool isVccOutOfRange()
{
  long vcc = readVcc();
  if ((vcc < 4500) || (vcc > 5500)) return true;  // ← 4,5 V ici
```

- `result == 0` (ADC bloqué à 0, référence défaillante) provoque une division par zéro. Sur AVR, `__udivmodsi4` renvoie une valeur indéterminée sans trap, mais c'est un comportement indéfini formellement. Ajouter `if (result == 0) return 0;`.
- Deux seuils de sous-tension coexistent : 2400 mV pour le bit QUES:VOLT, 4500 mV pour l'auto-test. Un Nano alimenté à 4,3 V échouera au `*TST?` mais n'armera pas le bit de statut. Unifiez via des constantes nommées (`VCC_MIN_MV`, `VCC_MAX_MV`).
- Enfin, `readVcc()` et `readTemp()` ne sont appelés que sur `:MEAS:…?` et `*TST?`. **Les bits QUES:VOLTage et QUES:TEMPerature ne sont donc jamais mis à jour spontanément.** Pour que la cascade ait un sens en surveillance, il faut un rafraîchissement périodique dans `loop()` (toutes les secondes suffisent, coût ~25 µs).

### B21 — Étalonnage du capteur de température non renseigné

**Lignes 1094–1095**

```cpp
float TS_GAIN   = 1.22;   // à corriger !
float TS_OFFSET = 324.31; // à corriger !
```

La dispersion du capteur interne de l'ATmega328P est spécifiée à ±10 °C sans étalonnage individuel (datasheet §24.8). Les bits QUES:TEMP (« CPU too hot » ≥ 50 °C, « CPU too cold » ≤ 10 °C) déclencheront donc à peu près arbitrairement. Deux options :

- **(a)** Étalonner en deux points et stocker `TS_GAIN`/`TS_OFFSET` dans le bloc de calibration EEPROM que vous avez déjà prévu — c'est la voie propre et cohérente avec `:CALibration:DATA`.
- **(b)** Tant que l'étalonnage n'est pas fait, ne pas armer les bits QUES:TEMP et faire retourner `9.91E37` (NaN) à `:MEAS:TEMP?`. Un contrôleur préfère une valeur explicitement invalide à une valeur fausse.

Je recommande (b) immédiatement puis (a) à terme. Notez que ces deux variables sont des `float` globaux non `const` : elles consomment 8 octets de RAM et empêchent le repliement de constante.

### B22 — `regBitSet`/`regBitClear` acceptent le bit 15

**Lignes 266 et 282** : `if (bit < 16)`.

Le bit 15 de tous les registres SCPI **doit toujours valoir 0** (il est réservé pour éviter les problèmes de signe sur les contrôleurs qui lisent en entier 16 bits signé). Les masques `& 0x7FFF` sont bien appliqués à `ena`, `ptr` et `ntr`, mais `con` et `eve` ne sont protégés que par les appelants. L'ancien code commenté (lignes 537–591) utilisait `bit < 15` : c'est une régression introduite lors de la refactorisation.

**Correctif** : `if (bit < 15)` aux deux endroits.

### B23 — Valeurs non initialisées lues dans `parseScpiRegister` / `:DIAGnostic:REGister`

**Lignes 388, 411–415** :

```cpp
int32_t bit;
int32_t event;
…
rc = isNR1(argv[0], bit);
if ((bit < 0) || (bit > 14))   // ← bit lu même si isNR1 a échoué (UB)
{
  rc = -222;                    // ← masque le vrai code d'erreur
}
```

Même schéma ligne 2498 pour `regaddr` (initialisé à `-1`, donc pas d'UB, mais l'erreur `-121` est écrasée par `-222`).

**Correctif** : initialiser (`int32_t bit = -1;`) et n'appliquer le test de plage que si `rc == 0`.

### B24 — Code mort : `if (regvalue < 0)` après `readRegister()`

**Lignes 2515–2519** : `readRegister()` retourne un `uint8_t`, la comparaison `< 0` est donc toujours fausse et la branche `-240` est inatteignable. `readRegister()` n'est par ailleurs **pas gardé par `isAs3935Available`** : `:DIAG:REG? 0` sur un capteur absent retourne du bruit SPI présenté comme une valeur valide.

### B25 — Statistiques : le paramètre `strike` de `readDistance()` est ignoré

**Lignes 1891–1936**

```cpp
void readDistance(bool strike)   // ← strike n'est jamais utilisé
{
  …
  if (lightning_distance < lightningDistanceMinimum) lightningDistanceMinimum = …;
  …
  lightningDistanceTotal += lightning_distance;
  distanceCount++;               // ← incrémente le compteur "distance", pas "lightning"
}
```

Comparez avec `readEnergy(bool strike)` (lignes 1955–2020) qui, lui, sépare correctement les deux jeux de statistiques. Conséquences :

- `:MEAS:DIST?` (appel avec `strike = false`) pollue `lightningDistanceMinimum/Maximum/Total`, réservés aux éclairs réels ;
- `:CALC:LIGH:DIST:AVER?` calcule `lightningDistanceTotal / lightningCount` alors que le total inclut des mesures hors éclair et que le compteur, lui, ne compte que les éclairs → **la moyenne est fausse par construction** ;
- les variables `distanceMinimum`, `distanceMaximum`, `distanceTotal`, `isDistanceOvfl` sont déclarées, réinitialisées par `resetStatistics(0x20)`… et jamais écrites ni lues par aucune commande. 17 octets de RAM et du code mort.

**Correctif** : aligner `readDistance()` sur `readEnergy()`, et soit implémenter `:CALCulate:DISTance:…` (symétrique de `:CALCulate:ENERgy:…`), soit supprimer le jeu de variables inutilisé.

### B26 — Débordements arithmétiques dans les statistiques

| Ligne | Expression | Risque |
|---|---|---|
| 1885 | `1000U * lightning_distance` | `unsigned int` = 16 bits sur AVR. 1000 × 40 = 40 000 (OK), mais 1000 × 63 = 63 000 (limite 65 535). Utiliser `1000UL`. |
| 2870 | `(1000UL * lightningDistanceTotal) / lightningCount` | Débordement 32 bits dès `lightningDistanceTotal > 4 294 967`, soit ~107 000 éclairs à 40 km. Calculer `1000UL * (total / count) + (1000UL * (total % count)) / count`. |
| 2805, 2966 | `(double)total / (double)count` | `double` = `float` (24 bits de mantisse) sur AVR : perte de précision au-delà de 16,7 M. Acceptable, mais à documenter. |

Le contrôle de débordement de `:CALC:LIGH:DIST:AVER?` est par ailleurs asymétrique : `:CALC:LIGH:ENER:AVER?` teste `isLightningEnergyOvfl || isLightningCountOvfl` (ligne 2799) alors que la version distance ne teste que `lightningCount == 0` (ligne 2864).

### B27 — Écriture EEPROM à chaque `*ESE` / `*SRE`

**Lignes 2089 et 2225** : `savePowerOnConfiguration()` est appelé à chaque écriture, y compris quand `isPSC == true` (cas où la configuration ne doit justement pas être restaurée au démarrage).

L'EEPROM de l'ATmega328P est donnée pour 100 000 cycles d'écriture. `EEPROM.put()` utilise `EEPROM.update()` en interne, donc seuls les octets modifiés sont réécrits — le risque est atténué mais réel pour un script de test qui balaie `*ESE 0` à `*ESE 255` en boucle.

**Correctif** : ne sauvegarder que si `isPSC == false` **et** si la valeur a changé ; ou mieux, différer l'écriture (marquer `confDirty = true` et écrire au plus une fois par minute depuis `loop()`).

### B28 — Somme de contrôle XOR trop faible

**Lignes 696–701**

```cpp
uint8_t computeChecksum(const uint8_t *p, const size_t s)
{
  uint8_t sum = p[0];
  for (size_t i = 2; i < s; i++) sum ^= p[i];
  return sum;
}
```

Un XOR d'octets ne détecte ni les permutations, ni les erreurs doubles sur un même rang de bit. Surtout, il donne des faux positifs sur les motifs dégénérés :

- **EEPROM vierge (0xFF)** : `SavedPonConf` fait 16 octets → `sum = 0xFF ^ (14 × 0xFF) = 0xFF` (nombre pair) = `poc.checksum` → **la somme de contrôle est validée**. Seul le test de version (`0x7F != 1`) rattrape le coup.
- **EEPROM à 0x00** : même chose, `sum = 0 == checksum`. Rattrapé par la version pour `SavedPonConf` et par le magic pour `SavedState`.

Le PDF parle explicitement de « Configuration CRC ». Un CRC-8 (polynôme 0x07 ou 0x31) coûte ~40 octets de Flash avec un calcul bit à bit, sans table :

```cpp
uint8_t crc8(const uint8_t *p, size_t n, size_t skip)
{
  uint8_t crc = 0xFF;                          // valeur initiale non nulle
  for (size_t i = 0; i < n; i++)
  {
    if (i == skip) continue;
    crc ^= p[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}
```

L'initialisation à 0xFF suffit à elle seule à éliminer le faux positif sur EEPROM vierge.

### B29 — `:SYSTem:UPTime?` : débordement à 49,7 jours

**Ligne 3300** : `Serial.print(millis()/1000UL);`

`isUptimeOvfl` est mis à `false` dans `setup()` et **jamais mis à jour**. `millis()` reboucle après 2³² ms ≈ 49,7 jours ; l'uptime repart alors à zéro sans avertissement.

**Correctif** :

```cpp
// dans loop()
static uint32_t lastMillis = 0;
uint32_t now = millis();
if (now < lastMillis) uptimeWraps++;     // uint8_t : 8 ans avant saturation
lastMillis = now;
// dans :SYSTem:UPTime?
uint32_t sec = (uint32_t)uptimeWraps * 4294967UL + now / 1000UL;
```

Avec un `uint8_t` pour `uptimeWraps`, `isUptimeOvfl` devient `uptimeWraps == 255`.

### B30 — `isBlank()` incomplet

**Lignes 3334–3337** — déjà identifié dans vos notes PDF.

```cpp
bool isBlank(char c)
{
  return (c == ' ') || (c == '\t');
}
```

IEEE 488.2 §7.4.1.2 définit le *white space* comme les codes ASCII 0–9 et 11–32 (soit tout caractère ≤ 32 sauf LF). Dans votre architecture, `\0` est le terminateur de chaîne C et `\n` le terminateur de message, donc :

```cpp
bool isBlank(char c)
{
  return (c > 0) && (c <= 32) && (c != '\n');
}
```

`\r` (13) devient alors un espace, ce qui est cohérent (il est déjà retiré en fin de ligne dans `loop()`). Attention : cette fonction est appelée dans la boucle de parsing des paramètres où elle sert aussi à valider ce qui suit une chaîne entre guillemets — vérifier que l'élargissement ne relâche pas ce contrôle.

### B31 — Divers

| Ligne | Point | Recommandation |
|---|---|---|
| 3314 | `:SYST:VERS?` retourne `"1990.0"` | Vous utilisez des fonctionnalités SCPI-99 (sous-registres QUEStionable, `:STATus:…:MAP`). Retourner `"1999.0"`. |
| 1066 | `displayError()` sans `default:` | Un code non répertorié (`-111`, `-123`, `111`) affiche `-111,""`. Ajouter `default: Serial.print(F("Error"));`. |
| 974–1066 | 4 entrées jamais générées (`-100`, `-200`, `-220`, `-314`) | ~90 octets de Flash récupérables. |
| 1552 | `-123` (*Exponent too large*) est généré mais n'a pas de libellé | Ajouter le `case`. |
| 2613 | `(subtoken == NULL) \| isToken(…)` | `\|` binaire au lieu de `\|\|` : pas de court-circuit, `isToken(NULL, …)` est évalué inutilement (il gère le cas, mais c'est fragile). |
| 2035–2041 | `Serial.print("NOIS")` etc. | Chaînes en RAM : 15 octets récupérables avec `F()`. |
| 1653 | `Serial.print(v?"1":"0")` | Idem : `Serial.write(v ? '1' : '0')`. |
| 339, 347 | `bitWrite(scpiStb, 5, scpiEsr & scpiEse)` | `bitWrite` est une macro ternaire dans le core actuel, donc correct — mais une valeur non booléenne est fragile si l'implémentation change. Normaliser : `((scpiEsr & scpiEse) != 0)`. |
| 170 | `lightningInt = 4` | Votre commentaire suggère D2 (INT0). Voir §10.1 et la contrainte matérielle de calibration en §9.5. |
| 42, 45, 48, 51 | `int noiseDefault` etc. non `const` | 8 octets de RAM récupérables en les passant en `const`. |
| 78–79 | `int16_t scpiQuesMap[15]` / `scpiOperMap[15]` | Seuls les bits 9 à 12 sont mappables (ligne 440) : `int16_t map[4]` suffit → **52 octets de RAM** récupérés. |

---

## 6. Conformité SCPI / IEEE 488.2

### 6.1 Commandes obligatoires

Les 13 commandes obligatoires IEEE 488.2 sont **toutes présentes** : `*CLS`, `*ESE`, `*ESE?`, `*ESR?`, `*IDN?`, `*OPC`, `*OPC?`, `*RST`, `*SRE`, `*SRE?`, `*STB?`, `*TST?`, `*WAI`. S'y ajoutent `*CAL?`, `*PSC`, `*RCL`, `*SAV`. C'est conforme.

Manquent (optionnelles, mentionnées dans vos notes) : `*PUD`/`*RUD`. Voir §9.8.

### 6.2 Points de non-conformité identifiés

| Réf. | Clause | Écart | Sévérité |
|---|---|---|---|
| B02 | 488.2 §10.3 | `*CLS` modifie ESE/SRE et n'efface pas l'ESR | Bloquant |
| B01 | SCPI-99 Vol.1 §9.4 | Bit TEMPerature mal placé | Bloquant |
| §6.3 | 488.2 §11.1.2 | RQS non distingué de MSS lors du serial poll | Majeur |
| §6.4 | SCPI-99 §7.2.1 | Un `<NRf>` envoyé à un paramètre entier doit être arrondi, pas rejeté | Majeur |
| B17 | SCPI-99 §7.2 | `PERCent` renvoie une fraction | Majeur |
| §6.5 | SCPI-99 §21.8 | Débordement du tampon d'entrée signalé `-112` au lieu de `-363` | Mineur |
| §6.6 | 488.2 §8.7 | `NAN`/`INF` en données caractère au lieu de `9.91E37`/`9.9E37` | À trancher |
| §6.7 | SCPI-99 §7.2.1 | `UP`/`DOWN` non implémentés | Fonctionnalité manquante |

### 6.3 RQS vs MSS lors du serial poll

**Lignes 3894–3901**

```cpp
if (c == 0x98)   // SPE
{
  updateStb();
  Serial.write(0x18);
  Serial.write(scpiStb);
  Serial.write(0x19);
  Serial.flush();
}
```

IEEE 488.2 §11.1.2 distingue deux significations du bit 6 :

- lu par **`*STB?`** → **MSS** (Master Summary Status), non destructif ;
- lu par **serial poll** → **RQS** (Request Service), qui doit être **remis à zéro après lecture**, tandis que MSS reste actif tant que la condition persiste.

Votre implémentation renvoie MSS dans les deux cas et ne remet jamais RQS à zéro. Un contrôleur qui boucle sur le serial poll verra un service request permanent.

**Correctif** :

```cpp
bool scpiRqs = false;

void updateStb()
{
  …
  bool mss = (scpiStb & (scpiSre & ~(1 << 6))) != 0;
  bitWrite(scpiStb, 6, mss);
  if (mss) scpiRqs = true;      // front montant → RQS armé
}

// serial poll :
updateStb();
uint8_t sb = scpiStb;
bitWrite(sb, 6, scpiRqs);
Serial.write(0x18); Serial.write(sb); Serial.write(0x19);
scpiRqs = false;                // RQS effacé par la lecture
```

**Remarque sur le choix de `0x98`** : utiliser un octet ≥ 0x80 comme échappement rend la liaison non transparente. Si vous implémentez un jour les blocs de données binaires (`#226…` évoqué dans le PDF), un octet 0x98 dans un bloc déclenchera un faux serial poll. Deux options : (a) réserver le serial poll à une séquence impossible en ASCII (par exemple `0x1B 0x18`), (b) documenter que les blocs binaires en entrée sont incompatibles avec le serial poll. Je recommande (a).

### 6.4 `<NR1>` refuse les formes NR2/NR3

**Lignes 1346–1441** : `isNR1()` rejette tout caractère non numérique, donc `.` et `E`.

SCPI-99 §7.2.1 impose qu'un paramètre déclaré entier accepte n'importe quelle forme numérique (`NR1`, `NR2`, `NR3`) et **arrondisse** à l'entier le plus proche. `:SENS:NOIS:THR 3.0` et `:SENS:NOIS:THR 3E0` doivent donc fonctionner et valoir 3. Actuellement ils retournent `-121`.

**Correctif structurel** : faire passer tous les paramètres numériques par `isNRf()`, puis arrondir pour les entiers :

```cpp
int16_t parseInteger(const char *s, int32_t &value)
{
  // Voie rapide : entier pur ou base #H/#Q/#B (pas de flottant impliqué)
  if (isPureInteger(s)) return isNR1(s, value);
  float f;
  int16_t rc = isNRf(s, f);
  if (rc == 0)
  {
    if ((f < -2147483648.0f) || (f > 2147483647.0f)) return -222;
    value = (int32_t)(f + (f < 0 ? -0.5f : 0.5f));
  }
  return rc;
}
```

La voie rapide évite de faire passer les cas courants par le flottant — ce qui compte si vous choisissez l'option « zéro flottant » du §8.1. Dans ce cas, écrivez un `parseInteger()` qui gère le point décimal et l'exposant **en arithmétique entière** (accumulation puis décalage décimal), ce qui reste plus petit que d'embarquer libm.

### 6.5 Débordement du tampon d'entrée

**Ligne 3931** : `scpiError = -112;` (*Program mnemonic too long*).

Ce code désigne un mnémonique de plus de 12 caractères, pas un tampon plein. Le code adapté est **`-363` (*Input buffer overrun*)**, ou à défaut `-102`. Par ailleurs, avec `SCPI_BUFFER_SIZE = 64`, un message légitime comme :

```
:STAT:QUES:LIGH:PTR 32767;:STAT:QUES:LIGH:NTR 32767
```

fait 51 caractères — la marge est étroite. Avec 52 octets de RAM récupérés sur les tableaux `map` (B31) et 8 sur les `const`, passer à `SCPI_BUFFER_SIZE = 96` ou 128 est finançable. `scpiBufferIndex` est un `uint8_t` : valide jusqu'à 255.

**Point connexe** : seul `\n` termine un message (ligne 3902). Un terminal configuré en CR seul (fréquent sous macOS ou avec certains outils de banc) ne déclenchera jamais l'exécution. Accepter les deux :

```cpp
else if ((c == '\n') || (c == '\r'))
{
  if (scpiBufferIndex == 0 && c == '\r') { /* ignorer CR isolé en tête */ }
  …
}
```

### 6.6 Représentation de NaN et de l'infini

**Lignes 954–964** : `SendNaN()` émet `"NAN"`, `SendInfinity()` émet `"INF"`.

Les constantes normatives sont pourtant déclarées (lignes 103–105) mais inutilisées :

```cpp
const float SCPI_PINFINITY =  9.9E37;
const float SCPI_NINFINITY = -9.9E37;
const float SCPI_NAN       =  9.91E37;
```

SCPI-99 §7.2.1 spécifie que dans une **réponse numérique**, NaN se représente par `9.91E37` et l'infini par `±9.9E37`. C'est ce que renvoient les instruments Keysight et Keithley. `NAN`/`INF` sont des données **caractère**, acceptables en *paramètre* d'entrée mais pas en réponse à une requête déclarée numérique.

L'enjeu est pratique : un contrôleur qui fait `float d = float(inst.query(':FETC:DIST?'))` plantera sur `"NAN"` et fonctionnera sur `9.91E37`.

**Recommandation** : émettre `9.91E37` et `9.9E37` en réponse, tout en continuant d'accepter `NAN`/`INF`/`NINF` en entrée (ce que fait déjà `isNan()`/`isInfinity()`). Comme ces deux littéraux sont fixes, ils s'écrivent avec `Serial.print(F("9.91E37"))` sans coût de formatage flottant — cohérent avec le §8.1.

### 6.7 `UP` / `DOWN` non implémentés

Les prédicats `isUp()` et `isDown()` existent (lignes 1256–1264) mais ne sont appelés nulle part. SCPI-99 définit `UP`/`DOWN` comme incrément/décrément du paramètre d'un pas, **sans erreur** quand la limite est atteinte (la valeur reste inchangée) — ce que votre PDF note correctement.

L'ajout est peu coûteux dans `checkInteger()` :

```cpp
else if (isUp(av[0]))   { v = value + step; if (v > valueMax) v = valueMax; }
else if (isDown(av[0])) { v = value - step; if (v < valueMin) v = valueMin; }
```

avec un paramètre `step` supplémentaire (1 par défaut). Cas particulier de `:SENS:LIGH:THR` dont les valeurs légales sont {1, 5, 9, 16} : `UP` doit passer à la valeur légale supérieure, pas à `valeur+1`. Un petit tableau PROGMEM `{1,5,9,16}` et une recherche d'indice résolvent le cas et servent aussi à valider l'écriture (remplaçant le test ligne 2429).

### 6.8 `:STATus:PRESet` — point à vérifier

**Lignes 250–262** : `regPreset()` applique `ENABle = 0`, `PTRansition = 0x7FFF`, `NTRansition = 0` à **tous** les registres, y compris les sous-registres VOLT/TEMP/CALI/LIGH.

C'est la convention appliquée par la plupart des instruments pour les registres OPERation et QUEStionable de premier niveau. En revanche, ma lecture de SCPI-99 Vol. 2 §20.4 est que les **sous-registres définis par SCPI** doivent recevoir `ENABle = tous les bits à 1`, précisément pour que « les événements dépendant de l'appareil soient rapportés à un niveau supérieur via la partie obligatoire du mécanisme de report d'état » (formulation du standard).

**Je ne peux pas vérifier ce point hors ligne** et vous invite à consulter le texte du standard, car il change complètement l'ergonomie : avec `ENABle = 0x7FFF` sur les sous-registres, l'utilisateur n'a plus qu'à armer `:STAT:QUES:ENAB` et `*SRE` pour recevoir les alarmes, au lieu de trois niveaux.

Second point, celui-ci sans ambiguïté : `regPreset()` **efface aussi le mapping** (`r.map[i] = 0`). Le mapping erreur→bit est une extension propriétaire ; `:STATus:PRESet` ne devrait pas le toucher, sous peine de rendre la commande destructrice de configuration. Je recommande de le déplacer vers `*RST` ou une commande dédiée `:STATus:QUEStionable:MAP:CLEar`.

### 6.9 Comportement après erreur dans un message composé

**Ligne 3353** : `while ((rc == 0) && isNotEol)` — dès qu'une commande retourne une erreur, le reste du message composé est abandonné. C'est **conforme** à IEEE 488.2 (§6.1.6.1 : le traitement du message programme s'arrête).

Un effet de bord mérite cependant attention : `*TST?` et `*CAL?` retournent `-330`/`-340` quand l'auto-test **échoue légitimement**. Un message `*TST?;:SYST:ERR?` abandonnerait donc la seconde requête après un test en échec. Or l'échec d'un auto-test est un résultat de mesure, pas une erreur de commande.

**Recommandation** : dissocier les deux notions.

```cpp
else if (isQuery && isToken(command, F("TST")))
{
  int16_t testRc = autoTest();       // affiche déjà le résultat
  if (testRc != 0) pushError(testRc);// journalise sans interrompre
  rc = 0;                            // la commande, elle, a réussi
}
```

---

## 7. Vérification de la cascade `:STAT:QUES:LIGH → QUES.10 → STB.3`

### 7.1 Chaîne nominale

```
  Événement matériel (éclair détecté)
        │
        ▼  regBitSet(lighReg, 0)
  ┌─────────────────────────────────────────┐
  │ QUES:LIGH:CONDition   bit 0 ── 0→1      │
  │        │  ET  QUES:LIGH:PTRansition.0   │  ← défaut 0x7FFF ✔
  │        ▼                                │
  │ QUES:LIGH:EVENt       bit 0 ── latch    │  ← effacé à la lecture
  │        │  ET  QUES:LIGH:ENABle          │  ← défaut 0x0000 ✘ à armer
  │        ▼  regCascade(lighReg)           │
  └────────┬────────────────────────────────┘
           ▼  updateQues() : regBitWrite(quesReg, 10, …)
  ┌─────────────────────────────────────────┐
  │ QUES:CONDition        bit 10            │
  │        │  ET  QUES:PTRansition.10       │  ← défaut 0x7FFF ✔
  │        ▼                                │
  │ QUES:EVENt            bit 10 ── latch   │  ← effacé à la lecture
  │        │  ET  QUES:ENABle.10 (=1024)    │  ← défaut 0x0000 ✘ à armer
  │        ▼  regCascade(quesReg)           │
  └────────┬────────────────────────────────┘
           ▼  updateStb() : bitWrite(scpiStb, 3, …)
  ┌─────────────────────────────────────────┐
  │ STB bit 3 (QUEStionable Summary)        │
  │        │  ET  SRE.3 (=8)                │  ← défaut 0x00 ✘ à armer
  │        ▼                                │
  │ STB bit 6 (MSS / RQS)                   │
  └─────────────────────────────────────────┘
```

**Verdict : la logique est correcte.** Le double filtrage transition→événement→activation est bien celui du modèle SCPI, l'ordre de calcul dans `updateStb()` est bon (le bit 6 est calculé en dernier, ligne 347), et le masquage à 0x7FFF interdit correctement le bit 15 sur `ena`/`ptr`/`ntr`.

Cinq problèmes viennent toutefois s'y greffer.

### 7.2 Problème C1 — Conflit entre le mapping d'erreurs et les bits de cascade

`parseScpiRegister()` autorise le mapping sur les bits **9 à 12** (ligne 440). Or le bit **10** est celui de la cascade LIGHtning. Séquence conflictuelle :

```
:STAT:QUES:MAP 10,-222      → mappe l'erreur -222 sur QUES bit 10
(une erreur -222 survient)  → pushError() : regBitSet(quesReg, 10)
                            → updateStb() → updateQues()
                            → regBitWrite(quesReg, 10, regCascade(lighReg))
                            → lighReg vide → regBitClear(quesReg, 10)
```

Le bit mappé est effacé **dans le même appel** qui vient de l'armer. Symétriquement, `clearOperQues()` (lignes 479–511) efface les bits 0, 3, 8, 10 gérés par la cascade.

**Solutions possibles**

| Option | Avantages | Inconvénients |
|---|---|---|
| **A.** Restreindre `MAP` aux bits 9, 11, 12 | Trivial (une ligne), supprime le conflit à la racine | Perd le bit 10 pour le mapping ; incohérent avec `:STAT:OPER:MAP` qui n'a pas de cascade |
| **B.** Masque `cascadeMask` par registre ; `MAP` et `clearOperQues()` l'excluent | Générique, extensible aux futurs sous-registres, cohérent OPER/QUES | ~40 octets de Flash, 2 octets de RAM par registre |
| **C.** Combiner en OU : `regBitWrite(quesReg, 10, cascade \|\| mapped)` | Aucune perte de fonctionnalité | Sémantique ambiguë pour le contrôleur : impossible de savoir *pourquoi* le bit est armé |

**Synthèse : retenir l'option B.** Elle résout aussi le cas symétrique de `clearOperQues()`, et elle reste correcte si vous ajoutez demain un sous-registre sur le bit 9 (WARNing) ou 13 (INSTrument). Ajoutez `uint16_t cascadeMask;` à `ScpiRegister` (`0x0511` pour `quesReg` = bits 0, 4, 8, 10 après correction de B01 ; `0x0000` pour les autres), puis :

```cpp
// dans parseScpiRegister, branche MAP en écriture
if ((bit < 9) || (bit > 12) || bitRead(r.cascadeMask, bit)) rc = -222;

// dans clearOperQues
if (bitRead(quesReg.cascadeMask, bit)) continue;
```

L'option A reste acceptable comme correctif immédiat en une ligne si vous voulez livrer vite.

### 7.3 Problème C2 — `updateStb()` n'est pas appelé après lecture d'un `:EVENt?`

**Lignes 375–385**

```cpp
else if (isQuery && ((subtoken == NULL) || isToken(subtoken, F("EVENt"))))
{
  rc = compareArgumentsCount(0, argc);
  if (rc == 0)
  {
    displaySeparator();
    Serial.print(r.eve);
    r.eve = 0;              // ← modifie l'état de la cascade
    scpiOutput = true;      // ← mais pas de updateStb()
  }
}
```

Après `:STAT:QUES:LIGH:EVEN?`, la variable `scpiStb` conserve son ancienne valeur. En pratique l'impact est limité — `*STB?` (ligne 2255) et le serial poll (ligne 3896) appellent tous deux `updateStb()` avant de lire — mais :

- `errorLed` (piloté ligne 329) reste dans un état obsolète ;
- si vous ajoutez un jour une **broche SRQ physique** ou une notification asynchrone, elle restera bloquée ;
- l'invariant « `scpiStb` reflète toujours l'état courant » est rompu, ce qui rend le débogage difficile.

**Correctif** : ajouter `updateStb();` après `r.eve = 0;`. Attention à la récursion (voir C3) : ajoutez d'abord la garde de réentrance.

Notez que la sémantique reste correcte après ce correctif : `updateQues()` recalculera `regCascade(lighReg) = 0` et effacera `QUES:CONDition` bit 10, mais `QUES:EVENt` bit 10 reste verrouillé (c'est un latch), donc STB.3 reste à 1 jusqu'à ce que `:STAT:QUES:EVEN?` soit lu à son tour. **C'est exactement le comportement attendu par le standard.**

### 7.4 Problème C3 — Récursion mutuelle non gardée

```
updateStb() → updateQues() → regBitWrite() → regBitSet() → updateStb() → …
```

La récursion **termine** : `regBitSet()` ne rappelle `updateStb()` que sur une transition 0→1 de `con`, et au second passage le bit est déjà à 1. La profondeur est bornée par le nombre de bits de cascade, soit 4, donc 5 niveaux au pire.

Mais chaque niveau empile 4 trames (`updateStb` + `updateQues` + `regBitWrite` + `regBitSet`), soit de l'ordre de **300 à 400 octets de pile** dans le pire cas. Sur 2 ko de RAM déjà occupés par ~600 octets de variables globales, les tampons série (2 × 64) et le tampon SCPI, la marge est réelle mais pas confortable — surtout si l'exécution se produit dans un contexte déjà profond (`pushError` appelé depuis `readEnergy` appelé depuis `loop`).

**Correctif** (également bénéfique en Flash — le compilateur peut inliner davantage) :

```cpp
void updateStb()
{
  static bool inUpdate = false;
  if (inUpdate) return;          // les appels imbriqués sont sans objet :
  inUpdate = true;               // la trame externe recalculera tout
  bitWrite(scpiStb, 2, (queueCount > 0));
  digitalWrite(errorLed, queueCount > 0);
  bitWrite(scpiStb, 3, updateQues());
  …
  inUpdate = false;
}
```

La correction est sûre : dans la trame externe, `updateQues()` effectue **tous** les `regBitWrite` avant que `regCascade(quesReg)` ne soit évalué en valeur de retour. L'état final est donc identique, pour un seul passage au lieu de cinq.

### 7.5 Problème C4 — Les bits d'événement sont des impulsions dans CONDition

**Lignes 3702–3706 et 3974–3998**

```cpp
regBitSet(lighReg, 0);       // Éclair détecté
digitalWrite(lightningLed, HIGH);
delay(5);
digitalWrite(lightningLed, LOW);
regBitClear(lighReg, 0);     // Fin éclair
```

Le bit 0 de `QUES:LIGH:CONDition` n'est haut que 5 ms. `:STAT:QUES:LIGH:COND?` ne le verra jamais en pratique. C'est cohérent avec le modèle (CONDition = état instantané, EVENt = latch), mais cela signifie que les bits 0, 8 et 9 sont **inutilisables en CONDition** et n'existent réellement qu'en EVENt.

Ce n'est pas un bug, mais c'est à documenter explicitement dans le manuel utilisateur : sinon un intégrateur qui interroge `:COND?` en boucle conclura que le détecteur ne fonctionne pas. À l'inverse, le bit 3 (« orage en cours ») est bien une condition durable — c'est le bon usage.

### 7.6 Problème C5 — Les registres VOLT et TEMP ne sont jamais rafraîchis

Comme signalé en B20, `regBitWrite(voltReg, …)` et `regBitWrite(tempReg, …)` ne s'exécutent que dans `readVcc()`/`readTemp()`, eux-mêmes appelés uniquement depuis `:MEAS:…?` et `*TST?`. Les branches 0 (VOLTage) et 4 (TEMPerature) de la cascade sont donc **inertes en surveillance continue** : un contrôleur qui arme `:STAT:QUES:ENAB 17; *SRE 8` puis attend un SRQ n'en recevra jamais, même si l'alimentation s'effondre.

**Correctif** : sonde périodique dans `loop()`.

```cpp
static uint32_t lastSense = 0;
if ((millis() - lastSense) >= 1000UL)
{
  lastSense = millis();
  readVcc();     // met à jour voltReg (bits 0 et 8)
  readTemp();    // met à jour tempReg (bits 0 et 8)
}
```

Coût : ~25 µs par seconde (les deux conversions ADC incluent `delay(2)` et `delay(20)` — à réduire, voir §10.1). C'est ce qui donne enfin un sens à l'ensemble de la structure QUEStionable.

### 7.7 Séquence de test recommandée

À exécuter ligne par ligne, réponses attendues en commentaire. Elle suppose B01, B02, C2 et C3 corrigés.

```scpi
*CLS                            # RAZ événements + file d'erreurs
:STAT:PRES                      # ENABle=0, PTR=0x7FFF, NTR=0 partout
:SYST:ERR?                      # → +0,"No error"

# --- Armement de la chaîne complète ---
:STAT:QUES:LIGH:ENAB 1          # bit 0 : "Lightning detected"
:STAT:QUES:ENAB 1024            # bit 10 : sommaire LIGHtning
*SRE 8                          # bit 3 : sommaire QUEStionable
:STAT:QUES:LIGH:ENAB?           # → 1
:STAT:QUES:ENAB?                # → 1024

# --- Vérification de l'état au repos ---
*STB?                           # → 0   (ou 4 si la file d'erreurs n'est pas vide)
:STAT:QUES:LIGH:COND?           # → 0
:STAT:QUES:COND?                # → 0

# --- Injection d'un éclair simulé à 10 km ---
:DIAG:LIGH 10

# --- Remontée de la cascade, du bas vers le haut ---
:STAT:QUES:LIGH:COND?           # → 8      (bit 3 : orage en cours ; bit 0 déjà retombé)
:STAT:QUES:COND?                # → 1024   (bit 10 armé par la cascade)
*STB?                           # → 72     (bit 3 = QUES + bit 6 = MSS)
:STAT:QUES:LIGH:EVEN?           # → 9      (bits 0 et 3 verrouillés) puis remis à 0
:STAT:QUES:LIGH:EVEN?           # → 0      (confirme l'effacement à la lecture)
*STB?                           # → 72     (QUES:EVEN bit 10 reste verrouillé) ✔
:STAT:QUES:EVEN?                # → 1024   puis remis à 0
*STB?                           # → 0      (toute la chaîne est retombée) ✔

# --- Vérification du filtre PTRansition ---
:STAT:PRES
:STAT:QUES:LIGH:PTR 0           # désarme les transitions montantes
:STAT:QUES:LIGH:NTR 8           # arme la transition descendante du bit 3
:STAT:QUES:LIGH:ENAB 8
:STAT:QUES:ENAB 1024
:DIAG:LIGH 5
:STAT:QUES:LIGH:EVEN?           # → 0   (aucune transition montante retenue) ✔
# provoquer un éloignement d'orage (distance = 63) pour armer le NTR
:STAT:QUES:LIGH:EVEN?           # → 8   (transition descendante retenue) ✔

# --- Vérification du bit 15 ---
:STAT:QUES:ENAB 65535
:STAT:QUES:ENAB?                # → 32767  (bit 15 forcé à 0) ✔

# --- Serial poll (octet 0x98) ---
# Après un éclair : doit retourner 0x48 avec RQS armé,
# puis 0x08 au poll suivant (RQS effacé, MSS maintenu).
```

Le point le plus discriminant est l'avant-dernier bloc `*STB? → 72` juste après avoir vidé `QUES:LIGH:EVEN` : il vérifie que le verrou du niveau parent est bien indépendant de celui de l'enfant. C'est le comportement que la plupart des implémentations maison ratent.

---

## 8. Réduction de l'empreinte Flash

Estimations pour avr-gcc à `-Os`. Elles sont indicatives (je n'ai pas pu compiler — §11) et à valider par un `avr-size` avant/après chaque étape.

### 8.1 Éliminer le virgule flottante — **gain estimé 2,5 à 3,5 ko** ⭐ priorité 1

C'est de loin le poste le plus lourd, et il est presque entièrement évitable.

**Ce que le flottant coûte aujourd'hui**

| Élément | Coût approximatif |
|---|---|
| `Serial.print(float/double)` → `dtostrf` + `__ftoa_engine` | ~1 700 octets |
| `sqrt()` (ligne 3714) → libm | ~300 octets |
| Arithmétique flottante (`__addsf3`, `__mulsf3`, `__divsf3`, `__fixsfsi`, `__floatsisf`) | ~1 200 octets |
| `isNRf()` + `checkFloat()` | ~700 octets (code applicatif) |

**Où le flottant est réellement utilisé**

| Usage | Ligne(s) | Alternative entière |
|---|---|---|
| Durée du buzzer en secondes | 3254, 3257 | Exposer `:SYST:BEEP:TIME` en **millisecondes** (`<NR1>`, 100–5000). Cohérent avec SCPI qui accepte les unités de base ; ou conserver les secondes en interne mais parser en millièmes. |
| Tension en volts | 2668 | `readVcc()` retourne déjà des **mV** (entier). Émettre en volts nécessite une virgule → formateur fixe (voir ci-dessous). |
| Énergie en pourcentage | 1946, 2761, … | Échelle entière (ppm ou centièmes de %). |
| Moyennes des statistiques | 2805, 2966 | Division entière + reste, ou moyenne × 100 en entier. |
| `sqrt()` pour la compression d'énergie | 3714–3716 | Racine carrée entière (~40 octets) ou table PROGMEM de 16 entrées. |

**Formateur décimal fixe** (~120 octets, remplace tous les `Serial.print(float)`) :

```cpp
// Émet une valeur entière mise à l'échelle, avec `dec` décimales.
// printFixed(4980, 3) → "4.980"   printFixed(1907, 4) → "0.1907"
void printFixed(int32_t scaled, uint8_t dec)
{
  if (scaled < 0) { Serial.write('-'); scaled = -scaled; }
  uint32_t div = 1;
  for (uint8_t i = 0; i < dec; i++) div *= 10;
  Serial.print(scaled / div);
  if (dec)
  {
    Serial.write('.');
    uint32_t frac = scaled % div;
    for (uint32_t d = div / 10; d > 1; d /= 10) if (frac < d) Serial.write('0');
    Serial.print(frac);
  }
}
```

Ce formateur donne en prime le contrôle sur le nombre de chiffres que `Serial.print()` ne permet pas, ce qui est **indispensable** pour respecter la spécification `:FORMat[:DATA] ASCii,<0..8>` de vos notes. C'est donc à la fois un gain de place et une fonctionnalité.

**Racine carrée entière** :

```cpp
uint16_t isqrt32(uint32_t n)
{
  uint32_t rem = 0, root = 0;
  for (int8_t i = 15; i >= 0; i--)
  {
    root <<= 1;
    rem = (rem << 2) | (n >> 30);
    n <<= 2;
    if (root < rem) { rem -= ++root; root++; }
  }
  return (uint16_t)(root >> 1);
}
```

**Attention** : le gain n'est acquis que si **plus aucune** opération flottante ne subsiste. Un seul `Serial.print(uneVariableFloat)` oublié ré-embarque `dtostrf` et annule ~1,7 ko. Vérifiez avec `avr-nm --size-sort` la disparition de `__ftoa_engine`, `dtostrf`, `__addsf3`.

### 8.2 Remplacer `snprintf()` — **gain estimé 1,3 à 1,5 ko** ⭐ priorité 1

**Ligne 692** : `snprintf(buildDateISO, sizeof(buildDateISO), "%s-%02u-%02uT%s", …)`

C'est le **seul** appel à la famille `printf` du fichier, et il embarque `vfprintf` en entier. Pour une date de compilation cosmétique, c'est le pire rapport bénéfice/coût du projet.

**Option A — suppression pure (gain 1,5 ko + 20 octets de RAM + ~200 octets pour `formatBuildDate()`)**

```cpp
// dans *IDN? :
Serial.print(F("ValTronix,LightningDetector,0," __DATE__ " " __TIME__));
```

La concaténation de littéraux est faite par le préprocesseur : la chaîne complète est en Flash, il n'y a plus ni tableau RAM ni fonction de formatage. Seul inconvénient : le format `"Aug 14 2026 13:27:00"` au lieu de l'ISO 8601. Pour une révision de firmware dans `*IDN?`, c'est parfaitement acceptable — d'autant que le standard n'impose aucun format pour ce quatrième champ.

**Option B — conserver l'ISO sans `snprintf` (gain 1,3 ko)** : construire la chaîne à la main avec des affectations de caractères (~80 octets).

**Synthèse : option A.** Si la date ISO vous tient à cœur pour un usage machine, préférez un vrai numéro de version sémantique (`1.2.0`) en constante `F()`, qui est de toute façon plus utile qu'une date de compilation pour un contrôleur.

Dans la foulée, `formatBuildDate()` utilise `strcmp` et `memcpy_P` sur la table `monthNames` (36 octets PROGMEM) : tout disparaît avec l'option A.

### 8.3 Supprimer la dépendance à `Wire` — **gain estimé 1 à 2 ko** ⭐ priorité 2

`SparkFun_AS3935.cpp` référence l'objet global `Wire` dans `beginI2C()`. L'éditeur de liens travaillant à la granularité du fichier objet, `TwoWire` **et** son pilote `twi.c` (avec ses tampons de 32 octets × 3 en RAM) sont embarqués même si vous n'utilisez que le SPI.

**Options**

| Option | Gain Flash | Gain RAM | Risque |
|---|---|---|---|
| **A.** Copier la bibliothèque dans le sketch et supprimer les méthodes I²C | 1–2 ko | ~96 octets | Perte du suivi amont (mais la bibliothèque est stable) |
| **B.** Réécrire un pilote SPI minimal (~15 fonctions) | 1,5–2,5 ko | ~96 octets | Effort de développement et de test |
| **C.** Ne rien faire | — | — | — |

**Synthèse : option A.** Vous avez déjà réimplémenté `readRegister()`/`writeRegister()` en SPI (lignes 854–881) pour `:DIAG:REG?` — l'essentiel du travail est fait. Copier `SparkFun_AS3935` dans le dossier du sketch, supprimer `beginI2C()`, `_i2cPort` et les branches `if (_i2cPort)` vous donne le gain immédiatement, et vous permettra ensuite d'exposer proprement les fonctions manquantes (`calibrateOsc`, `displayOscillator`, `tuneCap`) dont vous aurez besoin pour `:CALibration` (§9.5).

**À vérifier** : je n'ai pas la bibliothèque, ce gain est déduit de la structure habituelle de ce type de pilote. Mesurez avec `avr-size` sur un sketch minimal avant/après.

### 8.4 Factoriser le trio `displaySeparator` / `Serial.print` / `scpiOutput` — **gain estimé 600 à 900 octets** ⭐ priorité 2

Vous avez identifié le bon motif : il apparaît **environ 55 fois**.

```cpp
displaySeparator();
Serial.print(x);
scpiOutput = true;
```

Chaque occurrence coûte : 2 `call` (8 octets), le chargement de l'argument (2–6 octets), un `ldi`+`sts` pour le drapeau (4 octets), soit **14 à 18 octets**. Total actuel : ~900 octets. Avec un helper unique, chaque site tombe à un `call` + argument = 6 à 8 octets, plus ~60 octets pour les helpers.

```cpp
static void out(int32_t v)                       // entiers signés
{
  if (scpiOutput) Serial.write(';');
  Serial.print(v);
  scpiOutput = true;
}

static void outU(uint32_t v)                     // entiers non signés
{
  if (scpiOutput) Serial.write(';');
  Serial.print(v);
  scpiOutput = true;
}

static void outP(const __FlashStringHelper *s)   // littéraux (IND, OUT, NAN, LIGH…)
{
  if (scpiOutput) Serial.write(';');
  Serial.print(s);
  scpiOutput = true;
}
```

**Point important et non évident** : n'utilisez que **deux** types entiers. `Print::print()` possède des surcharges distinctes pour `int`, `unsigned int`, `long`, `unsigned long` ; `int`/`unsigned int` délèguent aux versions `long`, mais chaque type d'argument différent au site d'appel génère du code de conversion. En canalisant tout via `int32_t`/`uint32_t`, vous supprimez ces conversions dispersées. Ce détail vaut à lui seul 100 à 200 octets.

`SendNaN()` et `SendInfinity()` deviennent :

```cpp
#define sendNaN()      outP(F("9.91E37"))
#define sendInfinity() outP(F("9.9E37"))
```

Ce qui règle aussi le §6.6 sans coût supplémentaire.

**Variante encore plus compacte** pour les sites qui émettent une valeur puis positionnent un code d'erreur, très fréquents dans `:CALCulate` :

```cpp
// Émet v, ou NaN si invalid. Regroupe le motif "if (count==0) SendNaN(); else …"
static void outOrNaN(uint32_t v, bool invalid)
{
  if (scpiOutput) Serial.write(';');
  if (invalid) Serial.print(F("9.91E37")); else Serial.print(v);
  scpiOutput = true;
}
```

Ce motif seul apparaît 9 fois dans `:CALCulate` : ~150 octets supplémentaires.

### 8.5 Dispatch par table PROGMEM — **gain estimé 800 à 1 200 octets** ⭐ priorité 3

`processSCPISpecificCommand()` (lignes 2279–3332) est une cascade d'environ **75** `else if (isToken(subtoken, F("…")))`. Chaque test coûte :

- le littéral en PROGMEM (5 à 13 octets),
- la mise en place de l'argument + `call isToken` (8 à 10 octets),
- le test du résultat et le saut (4 à 6 octets),

soit 17 à 29 octets par nœud, environ **1,6 ko** au total pour la seule structure de décision.

**Point non évident** : `F("LIGHtning")` place la chaîne dans une section `.progmem.data` via un tableau statique anonyme. Contrairement aux littéraux `.rodata.str1.1`, **avr-gcc ne fusionne pas les duplicats**. Or le fichier contient `F("LIGHtning")` **6 fois**, `F("ENERgy")` 5 fois, `F("DISTance")` 4 fois, `F("STATe")`, `F("THReshold")`, `F("CLEar")`, `F("COUNt")`, `F("PERCent")` plusieurs fois chacun. À 8 octets en moyenne pour ~35 duplicats : **~280 octets gaspillés**.

**Correctif immédiat, sans restructuration** (~280 octets, 15 minutes de travail) : centraliser les mnémoniques.

```cpp
namespace Tok {
  const char LIGHTNING[] PROGMEM = "LIGHtning";
  const char ENERGY[]    PROGMEM = "ENERgy";
  const char DISTANCE[]  PROGMEM = "DISTance";
  const char VOLTAGE[]   PROGMEM = "VOLTage";
  …
}
#define FT(x) reinterpret_cast<const __FlashStringHelper *>(Tok::x)
// usage : isToken(subtoken, FT(LIGHTNING))
```

**Correctif structurel** (~800 à 1 200 octets, une demi-journée) : table plate de chemins complets.

```cpp
enum CmdId : uint8_t {
  CMD_SENS_AFE_GAIN, CMD_SENS_NOIS_THR, CMD_SENS_SPIK_REJ,
  CMD_CALC_LIGH_COUNT, CMD_CALC_LIGH_ENER_MIN, …, CMD_COUNT
};

struct CmdEntry { const char *path; uint8_t id; };   // path en PROGMEM

const char P_SENS_AFE[]  PROGMEM = "SENSe:AFE:GAIN";
const char P_SENS_NOIS[] PROGMEM = "SENSe:NOISe:THReshold";
…
const CmdEntry cmdTable[] PROGMEM = {
  { P_SENS_AFE,  CMD_SENS_AFE_GAIN },
  { P_SENS_NOIS, CMD_SENS_NOIS_THR },
  …
};

// Un seul comparateur qui traite le chemin complet, gère les nœuds
// optionnels ([:GAIN]) et les formes courtes/longues, puis :
switch (lookupCommand(command)) {
  case CMD_SENS_AFE_GAIN: … break;
  …
}
```

avr-gcc compile un `switch` sur des identifiants contigus en table de sauts (`ijmp`), soit ~4 octets par cas au lieu de 17 à 29. Le gain vient de la disparition de la structure de décision, pas des chaînes (qui restent nécessaires).

**Compromis pratique** : la difficulté est la gestion des **nœuds optionnels** (`[:GAIN]`, `[:COUNt]`, `[:PERCent]`, `[:CODE]`, `[:EVENt]`, `[:DC]`), qui multiplient les chemins. Une solution éprouvée consiste à noter l'optionalité dans la chaîne elle-même (par exemple par un marqueur `~` : `"SENSe:AFE~:GAIN"`) et à autoriser le comparateur à sauter les segments marqués. Cela ajoute ~60 octets au comparateur et supprime des dizaines de variantes.

**Synthèse** : commencez par la centralisation des mnémoniques (gain immédiat, risque nul). Ne passez à la table plate que si vous avez encore besoin de place après les points 8.1 à 8.4 — c'est la modification la plus risquée du lot et elle mérite une campagne de tests complète.

### 8.6 Factoriser les blocs `:CALCulate` — **gain estimé 500 à 800 octets** ⭐ priorité 3

Les blocs `:CALC:ENERgy:…` (lignes 2885–2984) et `:CALC:LIGHtning:ENERgy:…` (lignes 2738–2822) sont quasi identiques (MIN/MAX/AVER × PERCent). Les blocs `[:COUNt]?` / `:CLEar` sont répétés 5 fois à l'identique.

```cpp
struct Stat {
  uint32_t count, minimum, maximum, total;
  bool ovflValue, ovflCount;
};

Stat statLightning, statEnergy, statDistance, statDisturber, statNoise;

void statAccumulate(Stat &s, uint32_t v);
void statQuery(Stat &s, uint8_t what, uint32_t scale);  // 0=COUNT 1=MIN 2=MAX 3=AVER
void statReset(Stat &s);
```

Bénéfice secondaire non négligeable : les 25 variables globales `uint32_t` + 8 `bool` actuelles (lignes 205–231) deviennent 5 structures. La **RAM** y gagne peu (le contenu est le même) mais la lisibilité et la sûreté beaucoup — la plupart des bugs de statistiques signalés en B25/B26 viennent précisément de cette duplication.

### 8.7 `displayError()` — **gain estimé 250 à 350 octets** ⭐ priorité 4

Le `switch` de 30 cas (lignes 974–1066) coûte ~250 octets de structure de décision, plus ~620 octets de chaînes.

```cpp
struct ErrText { int16_t code; const char *text; };
const char E113[] PROGMEM = "Undefined header";
…
const ErrText errTable[] PROGMEM = { {-113, E113}, … };

void displayError(int16_t e)
{
  if (e >= 0) Serial.write('+');
  Serial.print(e);
  Serial.print(F(",\""));
  const char *t = NULL;
  for (uint8_t i = 0; i < ERR_COUNT; i++)
    if ((int16_t)pgm_read_word(&errTable[i].code) == e)
    { t = (const char *)pgm_read_word(&errTable[i].text); break; }
  Serial.print(t ? (const __FlashStringHelper *)t : F("Error"));
  Serial.write('"');
  scpiOutput = true;
}
```

Une boucle de recherche (~50 octets) remplace 250 octets de `switch`, et le `default` corrige au passage la lacune signalée en B31. En supprimant les 4 codes jamais générés, on récupère ~90 octets de plus.

### 8.8 Divers

| Action | Gain estimé |
|---|---|
| Supprimer `checkByte()` (redondant avec `checkInteger`) | ~150 octets |
| Supprimer `checkFloat()` si le flottant disparaît (§8.1) | ~250 octets |
| Supprimer les blocs commentés (lignes 536–620, 2091–2115, 2170–2183, 2203–2216, 2227–2251, 3765–3879) | 0 octet de Flash, gain de lisibilité important |
| `Serial.print(F("x"))` → `Serial.write('x')` pour les caractères isolés | ~10 octets par site |
| Passer le buzzer sur D3 (OC2B) et supprimer `tone()` | ~900 octets — **mais modification matérielle** |
| `int16_t map[15]` → `map[4]` (bits 9–12 seulement) | 0 Flash, **52 octets de RAM** |

### 8.9 Synthèse chiffrée

| Poste | Gain Flash | Effort | Risque |
|---|---|---|---|
| §8.1 Éliminer le flottant | 2 500 – 3 500 | Élevé | Moyen (touche les formats de réponse) |
| §8.2 Supprimer `snprintf` | 1 300 – 1 500 | **Très faible** | **Nul** |
| §8.3 Supprimer `Wire` | 1 000 – 2 000 | Faible | Faible |
| §8.4 Helper de sortie | 600 – 900 | Faible | **Nul** |
| §8.5 Table de dispatch | 800 – 1 200 | Élevé | Élevé |
| §8.6 Factorisation `:CALCulate` | 500 – 800 | Moyen | Faible |
| §8.7 Table d'erreurs | 250 – 350 | Faible | Nul |
| §8.8 Divers | 400 – 500 | Faible | Nul |
| **Total** | **7 350 – 10 750** | | |

**Ordre d'attaque recommandé** : 8.2 → 8.4 → 8.7 → 8.3 → 8.8 → 8.6 → 8.1 → 8.5.

Les quatre premiers rapportent **~3,5 ko pour une demi-journée de travail et un risque quasi nul**. Mesurez après chaque étape ; l'estimation ci-dessus est une projection, pas une mesure (§11).

---

## 9. Commandes SCPI proposées pour les fonctions AS3935 non exposées

### 9.1 Vue d'ensemble

| Fonction API | Registre | Exposée ? | Proposition |
|---|---|---|---|
| `powerDown()` / `wakeUp()` | `REG0x00[0]` PWD | ✘ | `:INITiate` / `:ABORt` (§9.2) |
| `clearStatistics()` | `REG0x02[6]` CL_STAT | ✘ | `[:SENSe]:LIGHtning:STATistics:CLEar` (§9.3) |
| `readInterruptReg()` | `REG0x03[3:0]` INT | ✔ | via `:FETCh:TYPE?` |
| `readDivRation()` / `changeDivRation()` | `REG0x03[7:6]` LCO_FDIV | ✘ | `:CALibration:DIVider` (§9.5) |
| `displayOscillator()` | `REG0x08[7:5]` DISP_* | ✘ | `:CALibration:SOURce` (§9.5) |
| `readTuneCap()` / `tuneCap()` | `REG0x08[3:0]` TUN_CAP | ✘ | `:CALibration:CAPacitance` (§9.5) |
| `calibrateOsc()` | `REG0x3D` CALIB_RCO | ✘ | `*CAL?` (§9.6) |
| `resetSettings()` | — | ✔ | via `*RST` |

### 9.2 Alimentation → modèle de déclenchement SCPI

`:INITiate` figure déjà comme stub dans votre code (lignes 2477–2488) et dans le PDF. C'est la bonne place pour `powerDown()`/`wakeUp()`.

**Justification** : le modèle de déclenchement SCPI (SCPI-99 Vol. 1 §5) définit trois états — IDLE, INITIATED, WAIT-FOR-TRIGGER — qui correspondent naturellement aux modes de l'AS3935 :

| État SCPI | Mode AS3935 | Commande |
|---|---|---|
| IDLE | Power-down (`PWD = 1`) | `:ABORt` |
| INITIATED / WAIT-FOR-TRIGGER | Listening | `:INITiate[:IMMediate]` |
| — | Listening permanent | `:INITiate:CONTinuous ON` |

```scpi
:INITiate[:IMMediate]            # wakeUp() + recalibrage RCO
:INITiate:CONTinuous <boolean>   # écoute permanente (valeur par défaut : ON)
:INITiate:CONTinuous?
:ABORt                           # powerDown()
```

C'est le vocabulaire utilisé par la quasi-totalité des instruments de mesure programmables pour l'armement d'acquisition, et il a un avantage décisif ici : il donne enfin un contenu au **registre `:STATus:OPERation`**, aujourd'hui structurellement présent mais toujours nul. Les bits SCPI-99 d'OPERation s'y prêtent directement :

- bit 4 **MEASuring** : le capteur est en écoute (post-`:INITiate`) ;
- bit 5 **WAITing for TRIGger** : idem, selon la granularité souhaitée ;
- bit 8–12 (définis par le concepteur) : par exemple bit 8 « calibration RCO en cours ».

**Point important** : `wakeUp()` déclenche un recalibrage des oscillateurs internes en fonction de la fréquence de résonance de l'antenne. La bibliothèque SparkFun retourne un booléen d'échec. Il faut donc :

```cpp
if (!lightning.wakeUp())
{
  regBitSet(caliReg, 1);   // "Timer RC Oscillator fail" (votre mapping PDF)
  regBitSet(caliReg, 3);   // "System RC Oscillator fail"
  rc = -340;               // Calibration failed
}
```

Cela alimente la branche `:STAT:QUES:CALibration` de votre cascade, aujourd'hui elle aussi toujours nulle.

**Alternative envisagée et écartée** : `:SYSTem:POWer ON|OFF`, employé par certains contrôleurs d'alimentation (dont l'EEZ Bench Box dont vos notes reprennent le format de `DIAG:TEST?`). Elle a le mérite de la simplicité, mais elle ne structure rien : `:INITiate`/`:ABORt` vous offrent en plus le registre OPERation et une voie d'évolution vers `:TRIGger` et `:READ?`.

> Réserve : je cite ces conventions constructeur de mémoire, sans pouvoir vérifier les manuels hors ligne. La justification par le modèle de déclenchement SCPI-99, elle, est indépendante de ces exemples.

### 9.3 Effacement des statistiques matérielles

L'AS3935 maintient **sa propre** mémoire d'événements (utilisée par l'estimateur statistique de distance), effaçable par bascule du bit `CL_STAT`. C'est distinct de vos compteurs logiciels effacés par `:CALCulate:*:CLEar`.

```scpi
[:SENSe]:LIGHtning:STATistics:CLEar     # event command, sans query
```

**Justification** : SCPI place sous `[:SENSe]` tout ce qui configure l'acquisition, et sous `:CALCulate` tout ce qui post-traite les données acquises (SCPI-99 Vol. 1 §4.1 — le modèle de mesure `SENSe → CALCulate → FORMat`). La mémoire statistique interne de l'AS3935 fait partie de la chaîne d'acquisition puisqu'elle influence directement la valeur de `DISTANCE` retournée par le capteur. La placer sous `:SENSe` est donc conforme au modèle, et évite toute confusion avec `:CALCulate:LIGHtning:CLEar` qui remet à zéro **vos** min/max/moyenne.

Documentez la distinction dans le manuel — c'est un piège classique pour l'intégrateur.

### 9.4 Sous-arbre `:CALibration` (accordage d'antenne)

Vos notes prévoient déjà une structure inspirée du Keithley 2001. Voici la proposition consolidée, cohérente avec les registres restants :

```scpi
:CALibration:PROTected:STATe <boolean>,"<password>"   # déverrouillage
:CALibration:PROTected:STATe?                         # → 0 (verrouillé) | 1
:CALibration:PROTected:CAPacitance <NR1>|MIN|MAX|DEF|UP|DOWN   # TUN_CAP 0-15
:CALibration:PROTected:CAPacitance? [MIN|MAX|DEF]
:CALibration:PROTected:CAPacitance:VALue?             # → capacité en pF (0..120)
:CALibration:PROTected:DIVider 16|32|64|128           # LCO_FDIV
:CALibration:PROTected:DIVider?
:CALibration:PROTected:SOURce LCO|SRCO|TRCO|NONE      # DISP_LCO/SRCO/TRCO sur IRQ
:CALibration:PROTected:SOURce?
:CALibration:PROTected:FREQuency?                     # fréquence LCO mesurée (Hz)
:CALibration:PROTected:AUTO ONCE                      # balayage automatique TUN_CAP
:CALibration:PROTected:STORe                          # écriture EEPROM
:CALibration:DATA?                                    # constantes d'étalonnage
:CALibration:DATE <yyyy>,<mm>,<dd>
:CALibration:DATE?
:CALibration:NDUE <yyyy>,<mm>,<dd>                    # prochaine échéance
:CALibration:NDUE?
```

**Justifications**

- `:CALibration:PROTected:…` avec verrouillage par mot de passe est la structure du **Keithley 2001** que vous citez, et le principe (constantes d'étalonnage protégées par un commutateur ou un mot de passe) est universel sur les instruments étalonnables.
- `ONCE` comme paramètre d'une commande `:AUTO` est la convention SCPI standard pour « exécuter une fois puis revenir en manuel » (SCPI-99 Vol. 2, §« AUTO ONCE ») — on la retrouve sur `:SENSe:VOLTage:RANGe:AUTO ONCE` de nombreux multimètres.
- `:CALibration:SOURce` pour router un signal interne vers une broche est cohérent avec l'usage de `SOURce` en SCPI. C'est le nommage sur lequel j'ai le moins de certitude : `:CALibration:OUTPut` serait aussi défendable.

**⚠️ Contrainte matérielle importante pour `:CALibration:FREQuency?` et `:AUTO ONCE`**

Mesurer la fréquence du LCO impose de compter les impulsions sur la broche IRQ. Or **D4 (votre `lightningInt`) n'est ni T1 ni ICP1** sur l'ATmega328P :

| Broche Nano | Fonction ATmega328P | Utilisable pour la mesure de fréquence |
|---|---|---|
| **D4** (actuelle) | T0 (entrée d'horloge Timer0) | ⚠️ Timer0 est utilisé par `millis()`/`delay()` — inexploitable |
| **D5** | T1 (entrée d'horloge Timer1) | ✔ Comptage matériel, précis |
| **D8** | ICP1 (capture Timer1) | ✔ Mesure de période, la plus précise |
| **D2** | INT0 | ✔ Comptage par interruption (jusqu'à ~50 kHz) |

Avec `LCO_FDIV = 128`, la fréquence sur IRQ est de 500 kHz / 128 ≈ 3,9 kHz — mesurable par interruption sur D2. Mais D2 est aussi le meilleur choix pour l'IRQ de détection (votre propre commentaire ligne 170).

**Synthèse** : passer l'IRQ sur **D8 (ICP1)** vous donne à la fois l'interruption sur changement (via PCINT0) et la capture matérielle pour la calibration, sans monopoliser INT0. Si vous préférez ne pas modifier le câblage, `:CALibration:FREQuency?` reste réalisable en échantillonnant `digitalRead()` dans une boucle serrée sur 100 ms (précision ~1 %), ce qui suffit pour choisir le meilleur `TUN_CAP` parmi 16 — l'objectif étant de trouver le maximum, pas de mesurer précisément.

### 9.5 Étalonnage RCO et `*CAL?`

Vos notes prévoient de rattacher `calibrateOsc()` à `*CAL?`, ce qui est conforme : IEEE 488.2 §10.2 définit `*CAL?` comme un auto-étalonnage interne retournant 0 en cas de succès.

```cpp
int16_t selfCalibration()
{
  uint8_t result = 0x00;
  if (!isAs3935Available) { out(1); return -241; }

  if (!lightning.calibrateOsc())
  {
    bitSet(result, 0);
    regBitSet(caliReg, 0);   // TRCO fail
    regBitSet(caliReg, 2);   // SRCO fail
  }
  out(result);
  return (result == 0) ? 0 : -340;
}
```

**Distinction essentielle à conserver** (vos notes la formulent déjà) : `*CAL?` recalibre les **oscillateurs RC internes** — opération volatile, refaite à chaque réveil — tandis que l'**accordage de l'antenne** (`TUN_CAP`) est un étalonnage manuel, stocké en EEPROM, avec date et échéance. Ne les mélangez pas : `*CAL?` ne doit jamais modifier `TUN_CAP`.

### 9.6 Commandes supplémentaires suggérées

```scpi
:SYSTem:SERial?                       # numéro de série depuis l'EEPROM (offset 0)
:FORMat[:DATA] ASCii[,<0..8>]         # nombre de chiffres de mantisse
:FORMat[:DATA]?
:SYSTem:TIME <h>,<m>,<s>              # si ajout d'un RTC
:SYSTem:DATE <y>,<m>,<d>
:STATus:QUEStionable:MAP:CLEar        # efface le mapping sans toucher au reste (§6.8)
```

`:SYSTem:SERial?` complète naturellement `*IDN?` qui renvoie aujourd'hui `0` en dur (ligne 2129) alors que votre plan EEPROM réserve les octets 0x00–0x0F au numéro de série et à son complément. Une fois lu, il devrait aussi alimenter le troisième champ de `*IDN?`.

`:FORMat` est déjà dans vos notes ; il n'a de sens qu'une fois le formateur du §8.1 écrit — les deux chantiers se rejoignent.

### 9.7 Suffixes numériques d'en-tête

Vos notes prévoient `BIT2`, `CHANnel1`. `isToken()` ne les gère pas, alors que le scanner d'en-tête (lignes 3499–3509) accepte déjà les chiffres après des lettres via le drapeau `numbersOnly`. L'ajout est donc localisé :

```cpp
bool isTokenN(const char *s, const __FlashStringHelper *tok, int16_t &suffix)
{
  size_t len = strlen(s);
  suffix = -1;                                  // -1 = absent
  while ((len > 0) && (s[len-1] >= '0') && (s[len-1] <= '9')) len--;
  if (len < strlen(s))
  {
    suffix = atoi(s + len);                     // ou une conversion maison
    char saved = ((char*)s)[len];
    ((char*)s)[len] = '\0';
    bool r = isToken(s, tok);
    ((char*)s)[len] = saved;
    return r;
  }
  return isToken(s, tok);
}
```

Le tampon étant modifiable, la restauration du caractère est sûre. Coût : ~80 octets. Utilité immédiate pour votre projet : `:STATus:QUEStionable:BIT10:ENABle` ne présente guère d'intérêt, mais `:SENSe:LIGHtning:THReshold` avec un suffixe de voie deviendra utile si vous ajoutez un second capteur.

**Recommandation** : reporter cette évolution. Le suffixe n'apporte rien tant qu'il n'y a qu'un capteur, et le gain de place est plus urgent.

### 9.8 `*PUD` / `*RUD`

`*PUD <bloc>` (Protected User Data) et `*RUD?` permettent de stocker une donnée utilisateur protégée (typiquement le numéro d'inventaire ou le lieu d'installation). Vos notes suggèrent d'y déplacer les paramètres du buzzer.

**Réserve** : `*PUD` prend obligatoirement un **bloc de données de longueur définie** (`#210HelloWorld`) — c'est spécifié par IEEE 488.2 §10.27. Vous ne pouvez donc pas l'implémenter sans traiter au préalable la syntaxe des blocs (déjà en étude d'après vos notes). Et le format bloc est mal adapté à un stockage structuré de paramètres de buzzer.

**Suggestion** : conservez les paramètres du buzzer dans le bloc `*SAV`/`*RCL` (où ils sont déjà et où ils ont leur place, puisqu'ils font partie de la configuration instrument) et réservez `*PUD` à sa vocation d'origine — une chaîne libre d'identification, par exemple `"Station meteo - toit batiment C"`. Ce sera aussi le premier consommateur légitime du parseur de blocs.

---

## 10. Améliorations fonctionnelles proposées

### 10.1 Supprimer les `delay()` du chemin critique

Recensement des blocages :

| Ligne | Blocage | Durée | Impact |
|---|---|---|---|
| 238 | `systemBeep()` | jusqu'à 5 000 ms | **Critique** — appelé depuis `pushError()` |
| 3234 | `:SYST:BEEP` | jusqu'à 5 000 ms | **Critique** |
| 3704, 3976, 3996 | Clignotement LED | 5 ms × 3 | Modéré |
| 3954 | Attente post-IRQ | 3 ms | Nécessaire (datasheet : 2 ms) |
| 1100 | `readTemp()` | 20 ms | Modéré |
| 1119 | `readVcc()` | 2 ms | Faible |
| 3885 | Bips de démarrage | 100 ms × n | Faible (setup uniquement) |

À 115200 bauds, le tampon RX matériel (64 octets) se remplit en **5,5 ms**. Tout blocage supérieur perd des caractères **sans qu'aucune erreur ne soit signalée** — le message tronqué produira au mieux un `-113` incompréhensible.

**Correctif recommandé** : machine à états pour les LED et le buzzer.

```cpp
struct Blink { uint8_t pin; uint32_t until; ScpiRegister *reg; uint8_t bit; };
Blink blinks[3];

void pulse(uint8_t idx, uint8_t pin, uint16_t ms, ScpiRegister *r, uint8_t bit)
{
  digitalWrite(pin, HIGH);
  if (r) regBitSet(*r, bit);
  blinks[idx] = { pin, millis() + ms, r, bit };
}

// dans loop() :
for (uint8_t i = 0; i < 3; i++)
  if (blinks[i].until && ((int32_t)(millis() - blinks[i].until) >= 0))
  {
    digitalWrite(blinks[i].pin, LOW);
    if (blinks[i].reg) regBitClear(*blinks[i].reg, blinks[i].bit);
    blinks[i].until = 0;
  }
```

Le `delay()` de `readTemp()` peut par ailleurs être supprimé en lançant la conversion ADC dans `loop()` et en relevant le résultat au tour suivant — la sonde périodique du §7.6 s'y prête naturellement.

### 10.2 Implémenter les bits « orage qui approche / qui s'éloigne »

Les bits 1 et 2 de `:STAT:QUES:LIGHtning` sont documentés dans votre schéma PDF mais ne sont **jamais armés** — uniquement effacés (lignes 1896–1898). C'est pourtant la fonctionnalité la plus intéressante du capteur pour un usage réel.

```cpp
uint8_t lastDistance = UINT8_MAX;

void updateStormTrend()
{
  if ((lightning_distance == 63) || (lightning_distance == 0))
  {
    regBitClear(lighReg, 1);
    regBitClear(lighReg, 2);
    regBitClear(lighReg, 3);
    lastDistance = UINT8_MAX;
    return;
  }
  if (lastDistance != UINT8_MAX)
  {
    regBitWrite(lighReg, 1, lightning_distance <  lastDistance);  // se rapproche
    regBitWrite(lighReg, 2, lightning_distance >  lastDistance);  // s'éloigne
  }
  lastDistance = lightning_distance;
  regBitSet(lighReg, 3);                                          // orage en cours
}
```

À appeler depuis `readDistance()` **et** depuis le cas `INT = 0` (B18), qui est précisément l'événement « la distance estimée a changé ». C'est aussi ce qui rendra le test NTR du §7.7 exécutable.

**Amélioration possible** : lisser sur 3 mesures pour éviter les oscillations, l'estimation de l'AS3935 étant bruitée par nature (algorithme statistique sur les 17 dernières minutes).

### 10.3 Ajouter un délai d'expiration d'orage

Le bit 3 (« orage en cours ») n'est effacé que si `readDistance()` renvoie 63 — ce qui suppose qu'une mesure soit déclenchée. Si l'orage cesse simplement, le bit reste armé indéfiniment.

```cpp
// dans loop()
if (bitRead(lighReg.con, 3) && ((millis() - lastStrikeMs) > 1800000UL))  // 30 min
{
  regBitClear(lighReg, 3);
  regBitClear(lighReg, 1);
  regBitClear(lighReg, 2);
}
```

30 minutes est cohérent avec la fenêtre statistique de 17 minutes de l'AS3935. À exposer éventuellement en `[:SENSe]:LIGHtning:TIMeout <NR1>` (secondes).

### 10.4 Protéger les commandes de diagnostic

`:DIAGnostic:LIGHtning` injecte de faux éclairs et pollue les statistiques. `:DIAGnostic:REGister` donne un accès brut au capteur. Ni l'une ni l'autre ne doit être accessible en exploitation.

**Suggestion** : les placer derrière le même verrou que la calibration.

```cpp
if (!calUnlocked) { rc = -203; }   // Command protected
```

Le code `-203` est déjà utilisé pour `:DIAG:REG` en écriture (ligne 2529) — la cohérence est donc immédiate. Ajoutez que l'injection doit marquer les statistiques comme non fiables, par exemple via un bit `:STATus:QUEStionable` bit 9 (WARNing), effacé au `*RST`.

### 10.5 Robustesse générale

- **Chien de garde** : `wdt_enable(WDTO_2S)` + `wdt_reset()` dans `loop()`. 40 octets pour une reprise automatique sur blocage. Attention : sur certains bootloaders Nano anciens (optiboot < 4.4), le chien de garde provoque une boucle de reset — testez.
- **Compteur de reboots** en EEPROM, exposé par `:DIAGnostic:INFOrmation:REBoots?` : très utile pour diagnostiquer un problème d'alimentation à distance.
- **`:SYSTem:ERRor:COUNt?` ne vérifie pas `argc`** (ligne 3103).
- **Détection de perte du capteur en fonctionnement** : relire périodiquement un registre connu et armer `:STAT:QUES:CALibration` en cas d'échec, plutôt que de ne tester qu'au démarrage.

### 10.6 Documentation

Trois commandes sont implémentées mais absentes de vos notes : `:MEASure:TEMPerature?`, `:DIAGnostic:LIGHtning`, et `*PSC`. Inversement, la liste du PDF contient des commandes marquées `*` (donc implémentées) que je n'ai pas trouvées dans le code — voir §11.

Un tableau unique « commande → état (implémentée / en étude / abandonnée) → ligne du source » vous éviterait cette dérive. Il serait aussi le point de départ naturel du fichier de tests du §12.

---

## 11. Ce que je n'ai pas pu analyser

**Bibliothèque `SparkFun_AS3935`** — non fournie. Je n'ai donc pas pu vérifier :

- les types de retour exacts de `readIndoorOutdoor()`, `readMaskDisturber()`, `readNoiseLevel()`, `readLightningThreshold()`, `distanceToStorm()`, `lightningEnergy()`. Le code mélange les usages (`readMaskDisturber() == 0` suppose un entier, `maskDisturber(state?0:1)` passe un entier à ce qui est probablement un `bool`) ;
- si `readLightningThreshold()` retourne le code registre (0–3) ou le nombre d'éclairs (1/5/9/16). Le code suppose la seconde forme (test ligne 2429) ; si c'est la première, `:SENS:LIGH:THR?` renvoie une valeur erronée. **À vérifier en priorité** ;
- si `lightningEnergy()` retourne bien les 21 bits assemblés (`REG0x06[4:0]`, `REG0x05`, `REG0x04`) et sur quel type ;
- si `beginSPI()` configure `pinMode(cs, OUTPUT)` — vos `readRegister()`/`writeRegister()` en dépendent ;
- l'ampleur réelle du gain du §8.3 (suppression de `Wire`).

**Compilation et mesure** — l'environnement d'analyse n'a ni toolchain AVR ni accès réseau. Je n'ai donc **aucun chiffre mesuré** :

- taille Flash et RAM actuelles, et donc la marge réellement disponible ;
- validation des gains estimés au §8 (ce sont des projections fondées sur les coûts typiques d'avr-gcc, pas des mesures) ;
- liste des avertissements `-Wall -Wextra -Wshadow`, qui contiendrait à coup sûr des points que j'ai pu manquer (B05, B06, B07 et les variables non initialisées en font partie) ;
- vérification que `bitWrite` est bien la macro ternaire dans votre version du core (§B31).

**Exécution et matériel** :

- comportement réel du parseur sur l'ensemble des messages composés (j'ai tracé une vingtaine de cas à la main ; B16 en est issu, mais d'autres chemins peuvent subsister) ;
- chronogrammes SPI, marges de timing, comportement de l'IRQ ;
- consommation de pile réelle (l'estimation du §7.4 est un calcul, pas une mesure — utilisez le motif de remplissage de pile pour la mesurer) ;
- fiabilité EEPROM et vérification des offsets à l'exécution.

**Documents** :

- le PDF fourni est une version aplatie ; les tableaux de registres apparaissent en partie sous forme d'images dont je n'ai lu que le rendu textuel. Certaines valeurs (notamment le tableau des seuils de bruit) peuvent avoir été altérées ;
- **incohérence non tranchée** entre le plan EEPROM du code (`CONF_BASE 0x010`, `SLOT_BASE 0x200`, `SLOT_SIZE 12`) et celui du PDF (configuration en 24–127, emplacements `*SAV` en 384–1023 avec 16 bits par entrée). Je ne sais pas lequel fait foi ;
- je n'ai pas pu consulter le texte de SCPI-99 ni d'IEEE 488.2 : mes références de clauses sont données de mémoire et **doivent être vérifiées**, en particulier le §6.8 sur `:STATus:PRESet` où l'enjeu ergonomique est réel ;
- de même, les précédents constructeurs cités au §9 (Keithley 2001, conventions `AUTO ONCE`) le sont de mémoire.

**Commandes du PDF marquées implémentées mais introuvables dans le code** — à confirmer, ma lecture a pu m'échapper :

- `:STATus:OPERation:*` : le parseur générique les accepte (ligne 3056), donc elles sont effectivement implémentées ✔
- `:CALCulate:ENERgy:CLEAr` : le PDF écrit `CLEAr` (A majuscule), le code teste `CLEar`. Simple coquille du PDF, sans effet.
- `:FETCh:TYPE[:CODE]?` : implémentée mais avec la sémantique du nœud optionnel inversée (voir ci-dessous).

**Point de sémantique à trancher** — `:FETCh:TYPE[:CODE]?` (ligne 2618) :

```cpp
fetchType(subtoken == NULL);   // NULL → true → sortie TEXTE
```

La notation `[:CODE]` signifie que `CODE` est le nœud **par défaut**, donc `:FETC:TYPE?` devrait être **identique** à `:FETC:TYPE:CODE?` et retourner le code numérique. Le code fait l'inverse. Deux corrections possibles :

- inverser l'implémentation (`fetchType(subtoken != NULL)`) — mais alors la forme texte devient inaccessible ;
- documenter `:FETCh:TYPE[:NAME]?` pour le texte et `:FETCh:TYPE:CODE?` pour le nombre. **Je recommande cette seconde voie** : la forme courte est la plus lisible pour un opérateur, et le contrôleur qui veut un nombre le demande explicitement.

---

## 12. Plan d'action recommandé

### Étape 0 — Prérequis (30 minutes, rentabilité maximale)

1. Activer `-Wall -Wextra -Wshadow` (*Préférences → Avertissements du compilateur → Tous*).
2. Relever la taille Flash/RAM de référence (`avr-size`) avant toute modification.
3. Créer un fichier de tests à partir des séquences du §7.7, exécutable par script.

### Étape 1 — Correctifs bloquants (une demi-journée)

B01 (bit TEMP) · B02 (`*CLS`) · B03 (traces de debug) · B04 (`checkByte`) · B05 (shadowing) · B06 (`rc = 0`) · B07 (`rc = 111`) · B08 (buzzer) · B14 (`argv`)

Tous sont des correctifs locaux, sans restructuration.

### Étape 2 — Correctifs majeurs (une journée)

B09 à B13, B15 à B17, plus C2 et C3 de la cascade (garde de réentrance et `updateStb()` après `:EVENt?`).

### Étape 3 — Gains Flash sans risque (une demi-journée, ~3,5 ko)

§8.2 (`snprintf`) → §8.4 (helper de sortie) → §8.7 (table d'erreurs) → §8.8 (divers) → §8.3 (`Wire`).

Mesurer après chaque sous-étape.

### Étape 4 — Cohérence de la cascade (une demi-journée)

C1 (masque `cascadeMask`) · C5 (sonde périodique VOLT/TEMP) · §10.2 (bits « approche / s'éloigne ») · B18 (`INT = 0`).

C'est l'étape qui rend le registre `:STATus:QUEStionable` réellement exploitable.

### Étape 5 — Conformité SCPI (une journée)

§6.3 (RQS) · §6.4 (`<NRf>` vers entier) · §6.5 (`-363` et CR/LF) · §6.6 (`9.91E37`) · §6.7 (`UP`/`DOWN`) · B11 (`VOLTage`).

### Étape 6 — Restructuration (selon la marge Flash restante)

§8.1 (élimination du flottant) · §8.6 (factorisation `:CALCulate`) · §8.5 (table de dispatch).

À n'entreprendre que si la place manque encore, et avec la campagne de tests de l'étape 0 en filet de sécurité.

### Étape 7 — Nouvelles fonctionnalités

§9.2 (`:INITiate`/`:ABORt` + registre OPERation) · §9.4 (`:CALibration`) · §10.1 (suppression des `delay`) · §10.3 (expiration d'orage) · §10.5 (chien de garde).

---

## Annexe A — Récapitulatif des lignes à modifier

| Ligne(s) | Réf. | Nature |
|---|---|---|
| 42, 45, 48, 51 | B31 | `const` manquant (RAM) |
| 78–79 | B31 | Tableaux `map` surdimensionnés |
| 266, 282 | B22 | `bit < 16` → `bit < 15` |
| 317 | **B01** | Bit 3 → bit 4 (TEMPerature) |
| 336 | **B12** | MAV toujours vrai |
| 339, 347 | B31 | Normalisation booléenne de `bitWrite` |
| 375–385 | **C2** | `updateStb()` manquant après `:EVENt?` |
| 388, 411–415 | B23 | Variables non initialisées |
| 440 | **C1** | Conflit MAP / cascade |
| 479–511 | **B05** | Shadowing de `i` |
| 514–525 | **B02** | `*CLS` non conforme |
| 696–701 | B28 | Somme de contrôle XOR |
| 692 | §8.2 | `snprintf` (1,5 ko) |
| 949–964 | §8.4, §6.6 | Helpers de sortie |
| 974–1066 | §8.7, B31 | Table d'erreurs + `default` |
| 1094–1095 | B21 | Étalonnage température |
| 1128–1137 | B20 | Division par zéro, seuils incohérents |
| 1584–1585 | **B03** | Traces de debug |
| 1586 | §6.4 | Exposant ignoré dans `isNRf` |
| 1693–1707 | **B04** | Affectation malgré erreur |
| 1885 | B26 | `1000U` → `1000UL` |
| 1891–1936 | B25 | Paramètre `strike` ignoré |
| 1946 | **B17** | Pourcentage / précision |
| 2515–2519 | B24 | Code mort, garde manquante |
| 2555 | **B06** | `(rc = 0)` |
| 2560–2563 | B06 | Exécution malgré l'erreur |
| 2618 | §11 | Nœud optionnel `[:CODE]` inversé |
| 2663, 3070 | **B11** | `VOLT` → `VOLTage` |
| 2870 | B26 | Débordement de la moyenne |
| 3050–3095 | **B10** | `else` final manquant |
| 3103 | §10.5 | `argc` non vérifié |
| 3205–3234 | **B08** | `argv[0]` → `argv[1]`, gardes |
| 3272–3277 | **B09** | Inversion `isDisturberMasked` |
| 3294–3300 | B29 | Débordement uptime |
| 3314 | B31 | Version SCPI |
| 3334–3337 | B30 | `isBlank()` |
| 3377, 3610–3614 | **B14** | Débordement `argv` |
| 3460, 3472 | **B07** | `rc = 111` |
| 3510–3528 | **B16** | `HEADER<espace>;` |
| 3726–3763 | **B13** | Ordre d'initialisation |
| 3922 | **B15** | Index de tampon |
| 3931 | §6.5 | `-112` → `-363` |
| 3894–3901 | §6.3 | RQS vs MSS |
| 4006–4016 | **B18** | `INT = 0` non géré |

*(en gras : bloquant ou majeur)*
