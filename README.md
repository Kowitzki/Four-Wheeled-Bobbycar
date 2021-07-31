# Formula-Student-Mini
This Repository contains information about the Formula Student Mini, a formula competition which is about small, extremely low cost and high powered vehicles.

I recommend to use Hoverboards, you get them used for ~30€ a piece, and they contain almost everything you need. This inclueds the Motors, which are inside the wheels, the Motorcontroller, which can drive two wheels and the accumulator.
The accumulators are pretty shitty, so not expect them to last very long. In a bobbycar they are overloaded. It helps if you put more of them in there. But you could use other types as well.

Keep in mind, this is illegal to drive on the street in many countries.

We are still working on this instruction, it is not near ready.


## Rules
We set up some rules. Some are for safety, some are to harmonize cars and some are for fun.
- Rules https://docs.google.com/document/d/1iLAwZntzIWLkrzUevK9PmmnD4W6sJKgPfaUV4WfNxIM/edit

# The black Bobbycar you saw at FSA 2021
There are many possible configurations to built a bobbycar like mine. So you can make changes to any part of this instruction. For example, if you want a better steering, you can change that part. Or if you do not want a Main Controller, you can use the Motorcontrollers directly connected to the potentiometers (throttle and brake).

The build process gets much easier if you take the rear wheel version instead of the four wheel version. You dont have to weld and you really dont need a seperate main controller. If you are not an electronics or mechanics expert, take the rear wheel version.
If you want to build this, keep in mind that 3kW is a lot of power and you can harm yourself or others easily. It does not have the safety a formula student car has, so please be careful at testing a new software. I highly recommend to implement a deadman switch, that ensures you really want the motors to spin.

## Software

STM32 ST-Link Utility:
From factory the Hoverboard Motorcontrollers are locked, so you have to unlock them by using the STM32 ST-Link Utility and a Flash Device called ST-Link V2. To use the ST-Link you have to install the Virtual COM Port Drivers.
- ST-Link V2 https://www.ebay.de/sch/i.html?_nkw=st-link+v2&_sop=12
- STM32 ST-Link Utility https://www.st.com/en/development-tools/stsw-link004.html
- Virtual COM Port Drivers https://www.st.com/en/development-tools/stsw-stm32102.html
- How To Unlock https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/wiki/How-to-Unlock-MCU-Flash


### Motor Controller
My preferred IDE for the Hoverboard Software is Visual Studio Code with PlatformIO. You have to restart your PC a few times during the installation, that helped me as things did not want to appear as supposed. You can set up the config.h as you like and you can even make changes to the main.c or other files to implement new features. Keep in mind that you should not delete safety features.
Be careful in Torque Mode if you want to get more speed. The MC Software has a bug at high speed. You can drive very fast in field weakening, but then, if you let go the throttle too fast, it brakes really hard. That will not happen if you set the field weakening current in config.h to a normal value like 5-6 Ampere. IF YOU SET IT HIGHER, TELL EVERYONE WHO DRIVES ABOUT THAT ISSUE. 


### Main Control
The Main Controller is programmed by the Arduino IDE with STM32duino. You have to make changes in the pin configuration, they are located at \AppData\Local\Arduino15\packages\stm32duino\hardware\STM32F4\2021.5.31\variants\blackpill_f401
In Line 57 you have to change PB7 to PB9. (#define BOARD_I2C1_SDA_PIN      PB9 //PB7)

Links:
- The used Software for Inverters, took out of Hoverboards https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
- The original Software for Inverters, took out of Hoverboards https://github.com/NiklasFauth/hoverboard-firmware-hack

## Electronics
STM32F4 Black Pill or STM32F1 Blue Pill
Bluetooth HC05
Display 1602 LCD
Steering Sensor AS5600
LAN Cable for communication and power to the steering wheel

Schematic for Electronics

Accumulator
I used two ebike accumulators from aliexpress. They have more voltage, which means a higher speed, and more capacity. Till now they last far better than the original ones.
Two of them have a capacity of about 550Wh, which lasts for 40km at about 20kmh and much coasting without braking.

Links:
- The Schematic of the Hoverboard Controllers https://beta.ivc.no/wiki/images/d/d7/Hoverboard_schematic_full_reverse.pdf
- STM32F4 BlackPill https://docs.zephyrproject.org/latest/boards/arm/blackpill_f411ce/doc/index.html
- STM32F4 BlackPill https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0.html

## Mechanics
The provided CAD Model is built in Autodesk Inventor, but there you can also find a step export.

### Frame
The Frame is built from 6mm and 21mm Plywood (Sperrholz), the kind with long fibres, not pressboard/particle board (Pressspan). Plywood is much more rigid.
There are dxf-files under "Mechanics" for lasercutting. Cutting by Hand is also possible, therefor you need an A3 printer and a jigsaw.
After cutting, 12mm parts are glued together out of 6mm pieces, then everything is pushed together and tightend by screws. Assembling the steering is not that easy, my bad.
If you want your wood to last longer in the rain i recommend to paint it.

### Steering
The uprights are printed using a 0.8mm or 1mm nozzle, a layer hight of 0.3mm and 60-80% infill. There are two options for the steering, one for normal driving and one for drifting. The options have a different geometry.
Unfortunately the steering shaft has to be welded, but i think every formula student team has someone in their team who can weld. Stick welding is sufficient.
If you use normal structural steel like S235 the steering shaft can be bend easily, already happend twice to me. Maybe you consider getting a tougher steel for the shaft.

## Drift Tires
Yeah, you read it right, i built drift tires for this. Fortunately a pipe in Germany named "Kanalgrundrohr" has the right size for this and is made out of PVC. A slice of this can be glued to printed PLA parts, which are mounted to the hoverboard motors using an TPU adaptor. This has been built, but not been tested yet. I am confident that it will work.


# Known Problems
- The steering is not going back to forward if you let go, and there is much friction. I already thought of putting bearings in there. Also a judge in FSA told me a spring could handle the caster problem.
- The Display is not working on the I2C Line, if the steeringsensor and the port expander for the brake light are used.
- If you have a four wheel car with two MCs, do not drive in field weakening when one controller is off. It would get more voltage than it is supposed to.

# Other Instructions to Bobbycars
- https://larsm.org/allrad-e-bobby-car/
- https://figch.de/index.php?nav=bobbycar

# Safety
- Do not charge unattended
- Do not delete safety features
- Do not drive on public streets
- Use a deadman switch for driving
- Think about what you want to do

# Beer
If you like my effort, i would appreciate if you get me a beer ;)
www.paypal.me/kowitzki/1.00
