# Sampling scheme

This page tries to document the sampling scheme combining the PWM timer, ADCs and DMA to sample the current sense at appropriate timing locations during a PWM cycle.
Please refer to the [pin map](PinMap.md) for a table over the different pins and their assigned peripherals.

See the [SyncedPWMADC](STM32-libraries/Periphirals/STM32G4/SyncedPWMADC) library for the implementation of the sampling scheme documented here together with all the STM32 peripheral configuration code.



In DC motor operation only 2 out of the 3 output channels are used. Using 2 of them results in an H-bridge configuration such that the motor can be driven in both directions.

Current sensing resistors are located on the low-side of the output channels and are thus only connected to the motor when the particular output is low. The corresponding current sense input to use therefore depends on the current direction the motor is being driven and it can only be sampled during the ON time of the low-side switching MOSFET.



## DC Motor driving

The H-bridge configuration can either be driven in Brake mode or Coast mode. The difference is in the OFF time of the PWM period where the all MOSFETs are disabled in Coast mode while both low-side MOSFETs are ON in Brake mode.

In the Brake mode configuration current can thus always be sensed through the low-side current sense on the motor terminal which connects to GND (obviously depending on direction). In Coast mode however the two motor terminals will be floating during the OFF time and there would be no voltage drop across the now disconnected current sense resistor.

The L6387ED chip is used to control the two MOSFETs of a single output channel and avoid surge current during switching. It has two inputs, a positive and complementary (negative) input. The intuitive way to think about the chip is basically that each input controls the state of each MOSFET and on top of that the chip adds the safety that both MOSFETs outputs can never be turned on at the same time.

When the two inputs to the L6387ED chip are different from eachother a single MOSFET will be turned on. If both inputs are low no MOSFETs are on. Finally if both inputs are high the chip will detect this error state and make sure that no MOSFETs are on.

### Brake mode

In Brake mode we force one of the motor channels to be always connected to GND by setting both inputs to the corresponding L6387ED chip to low. Then we control the other channel with PWM ensuring that the two inputs to the L6387ED are complementary and alternating.

### Coast mode

In Coast mode one of the motor channels is also always connected to GND by setting both inputs to the corresponding L6387ED chip to low. The difference is in the other actively driven channel where instead of driving both inputs to the L6387ED chip in a complementary setting, the low-side input is always LOW. This effectively disables the low-side MOSFET and the PWM input will thus only alternate between turning the high-side MOSFET ON and OFF.



## PWM Timer

We are using TIM1 which is an 16-bit Advanced-control timer which is used in Output compare configuration. The timer supports 4 channels which can be connected to output pins and two internal channels which can be used for triggering.

In our case we configure the timer in regular PWM mode (not center-aligned) and we configure 2 of the timer channels for the H-bridge outputs, configuring both the positive output and the complementary (negated) output. We furthermore configure two of the internal channels to specify two sampling instances during a PWM period. These two channels are combined into a Master output trigger which can then be used as a trigger for the ADC.

Another Master output trigger is also configured to the Update event (i.e. when the timer rolls over) which has only been using for experiments.

### Duty cycle

Positive duty cycle is defined as driving forward (driving direction = true). In this case TIM_CHANNEL_1 is set to LOW at all times such that TIM_CHANNEL_3 is the driving channel. In forward direction current sense is sensed through OPAMP1



# Sampling modes

The Timer, ADC and DMA periphiral are used jointly to enable high-frequency accurate sampling with as little CPU load as possible.

Three different sampling modes have been implemented:

1. Full-rate sampling: Sampling in every PWM cycle with interrupt every cycle (DMA transfer finished interrupt) - this imposes a limitation on the PWM frequency
2. Sampling triggered once every N PWM periods (where N >= 2)
3. Combination of the above two modes: Sampling triggered every N PWM periods with Full-rate sampling into a buffer storing M samples (where N >= 3 and M <= N-2). All the full rate samples are then averaged to yield the resulting sample.



## Sampling triggers

Up to two different trigger times are supported per PWM cycle. This is done by using the internal Capture Compare channel 5 and 6.

The two triggers can be positioned more or less freely within the PWM cycle, although they have to be sufficiently far apart to ensure that the ADC sampling will be complete for all configured channels before triggering again.

Three different duty-cycle synchronized triggering schemes have been implemented and tested with successful results:

1. Sampling **at the end of each ON+OFF period**, i.e. sampling right before each PWM edge, leads to measurements at the top and bottom of the 1-st order RL curve of the motor (inductance related)
2. Sampling **in the middle of each ON+OFF period** leads to an averaged current sensing 
3. Sampling **either in the middle of the ON or OFF period** depending on whichever period is longest

In either case the sampling time instance is configured such that the sampling of all configured channels are complete at the sample event mentioned above. This means that the sampling start trigger location is moved backwards from this event to compensate for the ADC sample time.

Anticipated ripple time (the settling time from switching the output) is also accounted for when computing the middle-type sampling and it is checked for to ensure that the sample start trigger location isn't configured to be before the ripple has settled.



## Sampling in every PWM cycle

In this mode the sampling runs completely autonomously. The DMA is configured in CIRCULAR mode with a length equal to the number of channels multiplied with the number of triggers.

The two timer triggers (CC5 and CC6) are used to trigger the ADC to sample the configured K number of channels (current sense, back emf, vbus etc.) 

In this mode the Timer Update interrupt (overflow) is not used.



## Sampling once every N-number of PWM periods

This sampling mode is achieved by setting the timers repetition counter which defines the rate at which preloaded timer values are reloaded but more importantly it defines the update (overflow/roll-over) interrupt generation rate which is used to initiate the sampling.



## Full-rate sampling with averaging once every N-number of PWM periods

In this mode the Timer update interrupt is used to initiate a DMA transfer



# ADC configuration

## Sample time and Oversampling





# DMA configuration





# Debugging

## Sampling pin

A sampling pin can be enabled to show when the ADC is sampling. The pin is set high with a capture compare interrupt defined at the same sampling instances and it is set low in the DMA sampling completed interrupt.