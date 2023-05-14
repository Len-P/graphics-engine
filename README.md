## Gequoteerde functionaliteit

V: Werkend  
-: Deels werkend met gekende problemen (onderaan beschreven)  
X: Niet werkend of niet geïmplementeerd  
(blanco): TODO


|   | Functionaliteit      | Status |
|---|---------------------------|--------|
| 1 | 2D L-systemen             | V      |
|   | Met haakjes               | V      |
|   | Stochastisch              |        |
| 2 | Transformaties            | V      |
|   | Eye-point                 | V      |
|   | Projectie                 | V      |
| 3 | Platonische Lichamen      | V      |
|   | Kegel en cylinder         | V      |
|   | Bol                       | V      |
|   | Torus                     | V      |
|   | 3D L-systemen             | V      |
| 4 | Z-buffering (lijnen)      | V      |
| 5 | Triangulatie              | V      |
|   | Z-buffering (driehoeken)  | V      |
| 6 | 3D fractalen              | V      |
|   | BuckyBall                 | V      |
|   | Mengerspons               | V      |
|   | View Frustum              |        |
| 7 | Ambient licht             | V      |
|   | Diffuus licht (oneindig)  | V      |
|   | Diffuus licht (puntbron)  | V      |
|   | Speculair licht           | V      |
| 8 | Schaduw                   | X       |
|   | Texture mapping           |        |
| 9 | Bollen en cylinders       |        |
|   | UV-coordinaten            |        |
|   | Cube mapping              |        |

Geïmplementeerde vorm van texture mapping: ...

## Gekende problemen
Schaduwen werken niet correct, omdat converteren van kleuren van doubles naar ints tussen 0-255 zorgt voor een verlies van precisie. (niet 100% zeker of het probleem hier ligt)
Er is echter van het begin gebruik gemaakt van doubles, pas wanneer de pixel wordt getekend stap ik over op integers.
Ik heb geen idee wat het probleem is, want ik werk op dezelfde manier bij alle andere delen van de engine en daar vormt dit geen probleem.
## Niet-gequoteerde functionaliteit
...

## Extra functionaliteit, niet in de opgaves beschreven
...

