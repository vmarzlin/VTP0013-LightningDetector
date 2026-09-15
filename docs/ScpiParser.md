# Grammaire générale

## En-tête

L'en-tête commence par:

- le caractère `*` suivi d'un seul mnémonique
- le caractère `:` suivi d'un ou plusieurs mnémoniques eux-même séparés par `:`. C'est alors un chemin absolu.
- directement par un ou plusieurs mnémoniques séparés entre-eux par `:`. C'est alors un chemin relatif.

**Note:** les espaces éventuels avant l'en-tête sont ignorés.

Un espace est defni par n'importe quel caractère ayant un code ASCII compris entre 0 et 32 à l'exception du caractère 10 (`\n`).

Chaque mnémonique commence forcément par une lettre (`A-Z` ou `a-z`). Il n'est pas sensible à la casse. Il peut aussi contenir, après la lettre initiale, des chiffres ou le symbole `_`. La longueur doit être de 12 caractères au maximum.

Si une en-tête termine par le caractère `?`, c'est une question; sinon c'est une commande.

### Grammaire des en-têtes

La grammaire est écrite au format EBNF (Extended Backus-Naur Form).

```bnf
(* Un header peut être : *)
<header> ::= "*" <mnemonic> ["?"] | [":"] <mnemonic> (":" <mnemonic>)* ["?"]

(* Un mnémonique commence par une lettre, suivie de 0 à 11 caractères autorisés *)
<mnemonic> ::= <letter> { <mnemonic-char> }{0,11}

(* Une lettre est une majuscule ou minuscule de A à Z *)
<letter> ::= "A" | "B" | "C" | "D" | "E" | "F" | "G" | "H" | "I" | "J" | "K" | "L" | "M"
            | "N" | "O" | "P" | "Q" | "R" | "S" | "T" | "U" | "V" | "W" | "X" | "Y" | "Z"
            | "a" | "b" | "c" | "d" | "e" | "f" | "g" | "h" | "i" | "j" | "k" | "l" | "m"
            | "n" | "o" | "p" | "q" | "r" | "s" | "t" | "u" | "v" | "w" | "x" | "y" | "z"

(* Un caractère de mnémonique peut être une lettre, un chiffre ou un souligné *)
<mnemonic-char> ::= <letter> | <digit> | "_"

(* Un chiffre est un caractère de 0 à 9 *)
<digit> ::= "0" | "1" | "2" | "3" | "4" | "5" | "6" | "7" | "8" | "9"
```

## Arguments

Une commande SCPI peut être suivi de 0 à n arguments. Chaque argument est séparé du suivant par une virgule ','. Il ne peut pas y avoir d'argument vide, donc "4,,8" par exemple est interdit.

Il existe différents types d'arguments comme les énumérations, les nombres, les chaînes de caractères entre guillemets ou encore les blocs.

Chacun de ces arguments sont listés en détail ci-dessous.

### Arguments de type enumération

Ces arguments sont des mots qui ne sont pas entre guillemets (`"`), ils sont lu exactement comme les mot-clé de l'entête. Ils respectent les mêmes règles: les parties en majuscules sont obligatoires, les parties en minuscules sont facultatives. Les mots possibles sont propres à chaque commande.

Exemple: `ACQuire:MODe SAMple`

Ici, l'argument `SAMple` aurait pu aussi être écrit `SAM` uniquement mais pas `SA` ni `SAMp` par exemple. La casse n'a pas d'importance. Ainsi, `sam`, `sample`, `SaMple` sont des orthographes valides.

### Arguments de type numérique

Beaucoup de commandes nécessitent des arguments numériques. La syntaxe ci-dessous montre le format que l'instrument renvoie en réponse à une requête. C'est aussi la format préféré lors de l'envoi de la commande à l'instrument, même si les formats seront acceptés. Cette documentation représente ces arguments comme décrit ci-dessous.

| Symbole | Signification |
| --- | --- |
| `<nr1>` | Nombre entier signé |
| `<nr2>` | Nombre flotant sans exposant |
| `<nr3>` | Nombre flotant avec un exponent (notation scientifique) |
| `<bin>` | Entier signé ou non au format binaire |
| `<oct>` | Entier signé ou non au format octal |
| `<hex>` | Entier signé ou non au hormat hexadécimal |

La plupart des arguments numériques seront automatiquement ramenés à une valeur valide — par arrondi ou par troncature — en cas de saisie d'un nombre invalide, sauf indication contraire dans la description de la commande.

Exemples de format valide pour les arguments numériques:

| `<nr1>` | `<nr2>` | `<nr3>` |
| --- | --- | --- |
| 123 | .123 | 1.23E+5 |
| +123 | +1.234 | 123.4E-56 |
| -12345 | -12345.6 | -12345.678E+90 |

Les nombres entiers `<nr1>` peuvent aussi être écrits en utilisant d'autres bases que la base 10. Il suffit pour cela de préfixer le nombre par `#` suivi d'un caractère qui va indiquer la base, suivi enfin par le nombre lui-même dans cette base. Note: ni le prefixe ni le nombre (en hexadécimal) n'est sensible à la casse.

| Symbole | Préfixe | Base |
| --- | --- | --- |
| `<bin>` | `#B` | binaire |
| `<oct>` | `#Q` | octal |
| `<hex>` | `#H` | hexadécimal |

Ainsi `#b1111` = `#Q17` = `#hF` = `#Hf` = 15.

Les formats `<bin>`, `<oct>` et `<hex>` sont une autre façon de saisir un entier au format `<NR1>`.
Le format `<NRf>` est le format numérique le plus flexible.  Un argument de type `<NRf>` peut indifféremment être au format `<NR1>`, `<NR2>` ou `<NR3>`. Si une commande attendait un nombre au format `<NR1>` (donc un nombre entier) et qu'elle reçoit un nombre au format `<NR3>` par exemple, ce nombre sera arrondi à l'entier le plus proche.

Le format flexible est très tolérant. Le signe d'exposant peut indiféremment être écrit en minuscule (`e`) ou majuscule (`E`). Il peut y avoir des espaces entre le signe et la mantisse ou l'exposant ainsi qu'autour du signe `E`. Ansi toutes les écritures suivantes sont valides: `.123`, `1.23`, `+12.3`, `.123E -45`, `.123 E + 1`, `+ 12.3e-1`.

### Arguments de type chaînes entre guillemets

Certaines commandes acceptent ou renvoient des données sous forme de chaîne entre guillemets, c'est-à-dire simplement un groupe de caractères ASCII délimité par des guillemets simples (`'`) ou doubles (`"`). Voici un exemple de chaîne entre guillemets : `"This is a quoted string"`. Cette documentation présente ces arguments de la manière suivante :

| Symbole | Signification |
| --- | --- |
| `<QString>` | Chaîne entre guillemets |

Une chaîne entre guillemets peut contenir n'importe quel caractère défini dans le jeu de caractères ASCII 7 bits. Respectez les règles suivantes lors de l'utilisation de chaînes entre guillemets :

1. Utilisez le même type de guillemet pour ouvrir et fermer la chaîne. Par exemple : `"this is a valid string"`.
2. Vous pouvez combiner différents types de guillemets au sein d'une chaîne, à condition de respecter la règle précédente. Par exemple : `"this is an 'acceptable' string"`.
3. Vous pouvez inclure un guillemet dans une chaîne en le répétant. Par exemple : `"here is a "" mark"`.
4. Les chaînes peuvent contenir des caractères majuscules ou minuscules.
5. Un caractère de retour chariot ou de saut de ligne inclus dans une chaîne entre guillemets ne met pas fin à la chaîne. Ce caractère est traité comme n'importe quel autre caractère de la chaîne.
6. La longueur maximale d'une chaîne entre guillemets renvoyée par une requête est de 1000 caractères.

Voici des exemples de chaînes de caractères invalides:

- `"Invalid string argument'` (les quillemets ne sont pas les mêmes)
- `"test` (les guillemets de fin sont manquants)

### Les arguments de type bloc

Certaines commandes utilisent une forme d'argument de type bloc pour définir une plage ou un type de valeur, comme indiqué dans le tableau ci-dessous.

| Symbole | Signification |
| --- | --- |
| `<digit-nz>` | Un chiffre non nul, donc dans la plage de 1 à 9 |
| `<digit>` | N'importe quel chiffre, donc dans la plage de 0 à 9 |
| `<octet>` | Un caractère avec une valeur hexadécimale équivalente comprise entre 00 et FF (0 à 255 en décimal) |
| `<bloc>` | Un bloc d'octets défini par: `<bloc> ::= {#<digit-nz><digit>[<digit>...][<octet>...]}` |

`<digit-nz>` indique le nombre d'éléments `<digit>` qui suivent. Pris ensemble, les éléments `<digit-nz>` et `<digit>` forment un entier décimal indiquant combien d'éléments `<octet>` suivent.

Exemple de bloc: `#223BANC-RF-R&D-ASSET-09411`

Ce même bloc aurait tout aussi bien pu être écrit: `#3023BANC-RF-R&D-ASSET-09411` ce qui est rigoureusement identique au bloc précédent.

Un bloc vide peut s'écrire `#10` (forme préférée) ou `#200`, etc.

### Grammaire des arguments

```bnf
(* Liste de paramètres : 0 ou plusieurs paramètres séparés par des virgules *)
<parameter-list> ::= <parameter> [ "," <parameter> ]*

(* Un paramètre peut être un booléen, un nombre, une chaîne, un bloc ou un mnémonique *)
<parameter> ::= <bool> | <nrf> | <qstring> | <bloc> | <mnemonic>

(* Booléen : "ON"/"OFF" (insensible à la casse) ou un nombre *)
<bool> ::= <bool-literal> | <nrf>

(* Booléen littéral : ON ou OFF, avec n'importe quelle combinaison de casse *)
<bool-literal> ::= ("O" | "o") ("N" | "n") | ("O" | "o") ("F" | "f") ("F" | "f")

(* Nombre au format libre : binaire, octal, hexadécimal, décimal, flottant, ou scientifique *)
<nrf> ::= <bin> | <oct> | <hex> | <nr1> | <nr2> | <nr3>

(* Binaire : #B ou #b suivi d'au moins un chiffre binaire *)
<bin> ::= ("#B" | "#b") { <digit-bin> }+

(* Octal : #Q ou #q suivi d'au moins un chiffre octal *)
<oct> ::= ("#Q" | "#q") { <digit-oct> }+

(* Hexadécimal : #H ou #h suivi d'au moins un chiffre hexadécimal *)
<hex> ::= ("#H" | "#h") { <digit-hex> }+

<digit-nz> ::= "1" | "2" | "3" | "4" | "5" | "6" | "7" | "8" | "9"

(* Chiffre binaire : 0 ou 1 *)
<digit-bin> ::= "0" | "1"

(* Chiffre octal : 0-7 *)
<digit-oct> ::= <digit-bin> | "2" | "3" | "4" | "5" | "6" | "7"

(* Chiffre hexadécimal : 0-9, A-F, a-f *)
<digit-hex> ::= <digit> | "A" | "B" | "C" | "D" | "E" | "F" | "a" | "b" | "c" | "d" | "e" | "f"

(* Nombre entier signé ou non *)
<nr1> ::= ["+" | "-"] { <digit> }+

(* Nombre flottant signé ou non *)
<nr2> ::= ["+" | "-"] ( { <digit> }+ "." { <digit> }* | "." { <digit> }+ )

(* Nombre scientifique signé ou non *)
<nr3> ::= ["+" | "-"] ( { <digit> }+ "." { <digit> }* | "." { <digit> }+ | { <digit> }+ ) ("E" | "e") ["+" | "-"] { <digit> }{1,9}

(* Chaîne de caractères entre guillemets simples ou doubles *)
<qstring> ::= <q1string> | <q2string>

(* Chaîne entre guillemets simples : les apostrophes sont doublées *)
<q1string> ::= "'" { <q1char> | "''" }* "'"
<q1char> ::= CHR(32) | CHR(33) | ... | CHR(126) | CHR(0) | CHR(1) | ... | CHR(31)  (* Tous les caractères ASCII, sauf ' *)

(* Chaîne entre guillemets doubles : les guillemets sont doublés *)
<q2string> ::= "\"" { <q2char> | "\"\"" }* "\""
<q2char> ::= CHR(32) | CHR(33) | ... | CHR(126) | CHR(0) | CHR(1) | ... | CHR(31)  (* Tous les caractères ASCII, sauf " *)

(* Bloc : # suivi de 1 à 10 chiffres, puis des octets quelconques *)
<bloc> ::= "#" <digit-nz> { <digit> }* {<octet>}*
<octet> ::= CHR(0) | CHR(1) | ... | CHR(255)
```

## Commandes enchaînées

Les commandes enchaînées sont au moins 1 en-tête, éventuellement suvi d'arguments. Chaque `en-entête [liste de paramètres]` doit être séparés par un point-virgule (`;`). Il est interdit d'avoir une en-tête vide. Donc par exemple ";;" est invalide.

### Grammaire des commandes enchaînées

```bnf
<cmd-list> ::= <header> [<space> <parameter-list>] [";" <cmd-list>]
<space> ::= <space-char> | <space> <space-char>
<space-char> ::= CHR(0)...CHR(9) | CHR(11)...CHR(32)
```
