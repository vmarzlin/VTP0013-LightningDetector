# Les codes d'erreur

Les erreurs sont stockées dans un pile d'erreurs/d'événements FIFO dédiée. S'il n'y a plus de place dans cette pile, le message le plus récent est remplacé par l'erreur -350 "Queue Overflow" et la queue n'accepte plus de nouveaux messages.

Tant que la queue contient au moins un message, le bit EAV du registre STB est à 1. La commande `*CLS` permet entre autre de vider totalement cette queue, ce qui aura pour conséquence de mettre aussi le bit EAV à 0. L'état du bit EAV est aussi généralement visible directement sur la façade de l'instrument via une LED rouge marquée "Erreur".

Les codes d'erreurs peuvent être affichés, et donc dépilés grâce à la commande `:SYSTem:ERRor[:NEXT]?`. Chaque erreur est alors affichée sous la forme `<code d'erreur>,"<message d'erreur>`. Le message d'erreur correspond au message standard correspondant au code, suivi éventuellement d'un message plus précis. Le message standard et le message complémentaire sont séparés par un point-virgule (`;`). Exemple:

```txt
-330,"Self-test failed;DAC out of range"
```

Note: les codes d'erreurs positifs ou nuls peuvent être précédés du signe plus ('+') mais ce n'est pas obligatoire. Ainsi le code d'erreur 0 peut âtre affiché de façon totalement équivalente sous ces deux formes:

```txt
+0,"No error"
0,"No error"
```

Les codes d'erreurs négatifs font partie du standard SCPI et sont découpés en différentes catégories selon le chiffre des centaines.

## Les codes standards

| Code | Message |
| --- | --- |
| 0 | +0,"No error" |
| -100 | -100,"Command error" |
| -101 | -101,"Invalid character" |
| -102 | -102,"Syntax error" |
| -103 | -103,"Invalid separator" |
| -108 | -108,"Parameter not allowed" |
| -109 | -109,"Missing parameter" |
| -110 | -110,"Command header error" |
| -112 | -112,"Program mnemonic too long" |
| -113 | -113,"Undefined header" |
| -120 | -120,"Numeric data error" |
| -121 | -121,"Invalid character in number" |
| -124 | -124,"Too many digits" |
| -200 | -200,"Execution error" |
| -220 | -220,"Parameter error" |
| -222 | -222,"Data out of range" |
| -224 | -224,"Illegal parameter value" |
| -233 | -233,"Invalid version" |
| -240 | -240,"Hardware error" |
| -241 | -241,"Hardware missing" |
| -310 | -310,"System error" |
| -311 | -311,"Memory error" |
| -313 | -313,"Calibration memory lost" |
| -314 | -314,"Save/recall memory lost" |
| -315 | -315,"Configuration memory lost" |
| -330 | -330,"Self-test failed" |
| -340 | -340,"Calibration failed" |
| -350 | -350,"Queue overflow" |
| -500 | -500,"Power on" |

### Les erreurs de commande

Ces erreurs vont de -100 à -199. À chaque erreur de ce type, le bit 5, CME (*CoMmand Error*), du registre ESR est mis à 1.

**-100,"Command error"**: 
**-101,"Invalid character"**: 
**-102,"Syntax error"**: 
**-103,"Invalid separator"**: 
**-108,"Parameter not allowed"**: 
**-109,"Missing parameter"**: 
**-110,"Command header error"**: 
**-112,"Program mnemonic too long"**: 
**-113,"Undefined header"**: 
**-120,"Numeric data error"**: 
**-121,"Invalid character in number"**: 
**-124,"Too many digits"**: 

### Les erreurs d'exécution

Ces erreurs vont de -200 à -299. À chaque erreur de ce type, le bit 4, EXE (*EXecution Error*), du registre ESR est mis à 1.

**-200,"Execution error"**: 
**-220,"Parameter error"**: 
**-222,"Data out of range"**: 
**-224,"Illegal parameter value"**: 
**-233,"Invalid version"**: 
**-240,"Hardware error"**: 
**-241,"Hardware missing"**: 

### Les erreurs de l'équipement

Ces erreurs vont de -300 à -399. À chaque erreur de ce type, le bit 3, DDE (*Device Dependent Error*) du registre ESR est mis à 1.

**-310,"System error"**: 
**-311,"Memory error"**: 
**-313,"Calibration memory lost"**: 
**-314,"Save/recall memory lost"**: 
**-315,"Configuration memory lost"**: 
**-330,"Self-test failed"**: 
**-340,"Calibration failed"**: 
**-350,"Queue overflow"**: 

### Les autres erreurs standards

**-500,"Power on"**: cette erreur est déclenchée à chaque fois que l'appareil est allumé. Cela permet de détecter qu'un instrument a redémarré suite à une coupure d'alimentation par exemple.

> Note: ce n'est pas dans le standard IEEE 488.2 qui lui, oblige uniquement à mettre à 1 le bit 7, PON (*Power ON*) du registre ESR. Le standard SCPI prévoit cette "erreur" -500 qui n'est implémentée que dans certains instruments mais ce n'est pas obligatoire.
