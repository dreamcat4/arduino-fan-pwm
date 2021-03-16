# arduino-fan-pwm

Combines other Arduino sketches, plus extra work. Towards a better fan pwm control, for arduino uno atmega328p

<!-- MarkdownTOC -->

* [Credits](#credits)
  * [Arduino Sketch - Initial Code](#arduino-sketch---initial-code)
  * [High frequency pwm on ATMega 328p, by maipulating timer registers](#high-frequency-pwm-on-atmega-328p-by-maipulating-timer-registers)
* [timers](#timers)
  * [atmega328p \(uno\)](#atmega328p-uno)
    * [timer0](#timer0)
    * [timer1](#timer1)
    * [timer2](#timer2)

<!-- /MarkdownTOC -->

<a id="credits"></a>
## Credits

This is a combined effort, after watching many educational videos explaining PID control algorithm. An also after gathering multiple arduino sketches / examples from around the web. And merging everything together.

<a id="arduino-sketch---initial-code"></a>
### Arduino Sketch - Initial Code

* Name: Chris Barnes
* Url: https://barnesian.com/arduino-powered-smart-fan-controller/
* License Terms: No idea, looks Public Domain license maybe. But Copyright © 2011-2017 Chris Barnes
* Comment: Lol. Can run a diff to see which lines of the original code remains

<a id="high-frequency-pwm-on-atmega-328p-by-maipulating-timer-registers"></a>
### High frequency pwm on ATMega 328p, by maipulating timer registers

* Name: Nick Gammon
* Url: http://www.gammon.com.au/forum/?id=11504
* License: copyleft 2005 DojoDave <http://www.0j0.org>
* Comment: Difficult to understand / follow but... Awesome stuff! Thanks so much & absolutely essential

<a id="timers"></a>
## timers

<a id="atmega328p-uno"></a>
### atmega328p (uno)

<a id="timer0"></a>
#### timer0

* used by `init()` function, called automatically by arduino IDE before `setup()`
* counts 1 time per `millisecond`, for the `millis()` funtion

<a id="timer1"></a>
#### timer1

<a id="timer2"></a>
#### timer2

* has a higher priority than timers 0 and 1







