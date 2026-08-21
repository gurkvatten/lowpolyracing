# Low Poly Racing

Prototyp för ett low-poly 3D racingspel med realistisk bilsimulering, byggt med
[raylib](https://www.raylib.com/) + C++.

## Bygga och köra

Kräver raylib installerat så att `pkg-config --libs raylib` fungerar
(makefilen faller annars tillbaka på vanliga standardsökvägar per OS).

**macOS:**
```
brew install raylib
make run
```

**Linux:** de flesta distros saknar raylib som paket, bygg från källkod:
```
git clone --depth 1 https://github.com/raysan5/raylib.git
cd raylib/src && make PLATFORM=PLATFORM_DESKTOP && sudo make install PLATFORM=PLATFORM_DESKTOP
cd ../.. 
make run
```
(kräver dev-paket för X11/OpenGL, t.ex. på Ubuntu:
`sudo apt install libgl1-mesa-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev`)

## Kontroller

- **W / ↑** – gas
- **S / ↓** – broms (bromsar till stillastående, sen back)
- **A/D eller ←/→** – styr
- **Mellanslag** – handbroms

## Fysikmodellen

Bilen simuleras som en "bicycle model" (fram- och bakaxel behandlas som en
punkt vardera, inte fyra oberoende hjul) i `src/car_physics.{h,cpp}`, helt
frikopplad från rendering i `src/main.cpp`. Det ger seriös
grepp/sladd-känsla utan att behöva en full fysikmotor.

Vad som simuleras:

- **Slip-vinkel-baserad däckkraft** via en förenklad Pacejka/magic-formula-
  kurva (stiger linjärt, planar ut, ger realistisk över-/understeer).
- **Viktöverföring** längsgående (broms/gas flyttar last mellan axlarna,
  vilket ändrar hur mycket grepp respektive axel har).
- **Friktionscirkel** per axel – fullgas eller hård inbromsning äter upp
  sidogrepp, precis som i verkligheten (går att tappa bak i en kurva genom
  att gasa för hårt).
- **Gir-dynamik** (yaw rate) härledd korrekt från krafter i ett roterande
  kroppsfast koordinatsystem (Coriolis-kopplingen `dvx/dt = Fx/m - ω·vz`,
  `dvz/dt = Fz/m + ω·vx` – se kommentarer i `car_physics.cpp`).
- **Motor som är både vridmoment- och effektbegränsad** (`F = min(maxKraft,
  maxEffekt / fart)`), så toppfarten blir realistisk istället för oändlig.
- **Luftmotstånd** (kvadratiskt) och **rullmotstånd**.
- **Fartkänslig styrning** (mindre max-styrvinkel vid hög fart) samt en
  kosmetisk lut/pitch-effekt vid kurvtagning/broms (inte del av
  rigid-body-simuleringen, bara för känsla).

Alla parametrar (massa, hjulbas, tyngdpunktshöjd, däckgrepp, motorkraft,
osv.) finns samlade i `CarConfig` i `car_physics.h` och är tänkta att
justeras/tunas.

### Kända begränsningar / naturliga nästa steg

- **Två axlar, inte fyra oberoende hjul.** Ingen per-hjul-fjädring eller
  independent slip vänster/höger. Nästa steg för djupare realism: fyra
  hjul med egen fjädring (spring-damper, ev. raycast mot marken) och
  individuell slip-ratio-baserad längsgående kraft.
- **Ingen växellåda/motorkurva.** Motorn modelleras som en enkel
  vridmoment+effekt-gräns, inte faktiska växlar/RPM/momentkurva.
- **Ingen kollisionsdetektering** mot bana/hinder ännu, och ingen bana –
  bara ett oändligt rutnät.
- **Ingen low-poly-konst** – bilen är just nu en enkel låda. Modeller
  (bil, bana, miljö) är ett eget spår att ta vidare.

## Filstruktur

```
src/
  main.cpp         - fönster, indata, kamera, rendering
  car_physics.h/.cpp - fysiksimulering (fristående från raylib-rendering)
makefile           - cross-platform (Linux/macOS)
```
