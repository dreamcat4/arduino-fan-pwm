# arduino-fan-pwm

Combines other Arduino sketches, plus extra work. Towards a better fan pwm control, for arduino uno atmega328p

<!-- MarkdownTOC -->

* [pid parameters](#pid-parameters)
  * [p = proportional](#p--proportional)
  * [i = integral = final little 'push' to nudge the temperature onto target](#i--integral--final-little-push-to-nudge-the-temperature-onto-target)
    * [tau_i](#tau_i)
    * [integral wind up](#integral-wind-up)
    * [integral clamping](#integral-clamping)
  * [d = derivative = puts on the brakes](#d--derivative--puts-on-the-brakes)
    * [derivative - high freq noise](#derivative---high-freq-noise)
    * [derivative - low pass cutoff filter](#derivative---low-pass-cutoff-filter)
    * [derivative term - laplace domain tranfer function](#derivative-term---laplace-domain-tranfer-function)

<!-- /MarkdownTOC -->


<a id="pid-parameters"></a>
## pid parameters

<a id="p--proportional"></a>
### p = proportional

p = proportional band = %pb = 100/pb * error
  pb = 100/p

Kp = gain = p = higher the faster the proportional response
 for example a gain of 18x means the output control variable (cv) will be 18*error

wheras proportional band = pb or kp = 100/p
then a lower value is higher gain... 1/18th as a % = 5.55%... so cv=100*error/pb

<a id="i--integral--final-little-push-to-nudge-the-temperature-onto-target"></a>
### i = integral = final little 'push' to nudge the temperature onto target

integral notices if the error value remains uncorrected
this occurs once the error gets close enough to the sepoint, 
that the proportional value is no longer strong enough to give a sufficient
positive enough value to overcome the inertia threshold for the last little bit
for example the static friction in a movement system, or other signal inertia

Ki = the integral constant = integral Gain
however very often there is no seperate term for the Integral Gain,
instead it's all rolled into the shared common gain 'K', when  Kp = Ki

<a id="tau_i"></a>
#### tau_i

however there is usually the 'inegral remembering time period'

integral reset = Tau_i

i = reset, can be defined in:
repeats per sec = hz
seconds per repeat
repeats per min
minutes per repeat

the integral = sum of area under curve
Ki, or Ti. where Ki = 1/Ti

**what is Tau_i?**

it's the term that we successively divide the Ki by
for example:

Ki/Tau_i * e
we need a working example of this
but basically the integral keeps forgetting the older terms each sample

the integral is only remembered since the last time period
it's like a sliding window
for example an integral's reset period of '6rpm'
means that it only remembers the last 10 seconds worth of error

<a id="integral-wind-up"></a>
#### integral wind up

occurs when the output motor is saturated
and cannot spin faster than a given rpm

this causes the error (e) to remain high, despite the output driving value being high
so then the integral keeps adding up

<a id="integral-clamping"></a>
#### integral clamping

to combat this issue, a special loop will impose limit on the integral value
this occurs whenever the output value exceeds the threshold, then we will clamp
once clamping is triggered, then the error term for the integral (Ei) will be set to 0
so long as the sign +- of the error term is the same as the sign of the output
which clears the integral value, and makes it 0, until clamping is released again
which occurs if there is no longer saturation, or if there is overshoot and positional error sign changes

integral clamping limit should be set lower than the maximum rpm of the output motor when saturated


<a id="d--derivative--puts-on-the-brakes"></a>
### d = derivative = puts on the brakes

d = d/dt = dpv/dt
the rate of change of the perceived value, over time
so this is how fast the rate the system is currently changing it's temperature at
this is governed by the responsiveness of the system

sp = set point = target temp
pv = process variable / perceived value = current temp
cv = control variable = mv (manupulated variable) = output to heater
e = error = (set point - perceived value) = distance to target


<a id="derivative---high-freq-noise"></a>
#### derivative - high freq noise

sensitive to noise spikes (high dt), which induce unwanted oscillations

<a id="derivative---low-pass-cutoff-filter"></a>
#### derivative - low pass cutoff filter

d/dt derivative is laeger for hf noise spikes
so we need to use a low pass filter on the derivative fumctiom

tuning the filter freq is an important variable

<a id="derivative-term---laplace-domain-tranfer-function"></a>
#### derivative term - laplace domain tranfer function

from 'undersstanding pid control part 3' video, at 10m45s

to incorporate both the derivative + low pass filter
we can instead create a negative feedback loop with the integral

this is more computationally effecient transfer function
combining the 2 blocks together.

