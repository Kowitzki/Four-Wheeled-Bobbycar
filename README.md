# The black Bobbycar you saw at FSA 2021
There are many possible configurations to built a bobbycar like mine. So you can make changes to any part of this instruction. For example, if you want a better steering, you can change that part. Or if you do not want a Main Controller, you can use the Motorcontrollers directly connected to the potentiometers (throttle and brake).

I recommend to use Hoverboards, you get them used for ~30€ a piece, and they contain almost everything you need. This includes the Motors, which are inside the wheels, the Motorcontroller, which can drive two wheels and the accumulator.
The accumulators are pretty shitty, so not expect them to last very long. In a bobbycar they are overloaded. It helps if you put more of them in there. But you could use other types as well.

The build process gets much easier if you take the rear wheel version instead of the four wheel version. You dont have to weld and you really dont need a seperate main controller. If you are not an electronics or mechanics expert, take the rear wheel version. You find it here: https://github.com/Kowitzki/Rear-Wheeled-Bobbycar  
If you want to build something like this, keep in mind that 3kW is a lot of power and you can harm yourself or others easily. It does not have the safety a formula student car has, so please be careful at testing a new software. I highly recommend to implement a deadman switch, that ensures you really want the motors to spin.

Keep in mind, this is illegal to drive on the street in many countries.

## Software

### Motor Controller
Emanuel Feru wrote a FOC Firmware for the Hoverboard Controllers. This is the one you should flash, but you can also take the original from Niklas Fauth.

Links:
- The used Software for Inverters, took out of Hoverboards https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
- The original Software for Inverters, took out of Hoverboards https://github.com/NiklasFauth/hoverboard-firmware-hack

My preferred IDE for the Hoverboard Software is Visual Studio Code with PlatformIO. You have to restart your PC a few times during the installation, that helped me as things did not want to appear as supposed. You can set up the config.h as you like and you can even make changes to the main.c or other files to implement new features. Keep in mind that you should not delete safety features.
Be careful in Torque Mode if you want to get more speed. The MC Software has a bug at high speed. You can drive very fast in field weakening, but then, if you let go the throttle too fast, it brakes really hard. That will not happen if you set the field weakening current in config.h to a normal value like 5-6 Ampere. IF YOU SET IT HIGHER, TELL EVERYONE WHO DRIVES ABOUT THAT ISSUE. 

### Main Control
The Main Controller is programmed by the Arduino IDE with STM32duino. I will upload the software for that but wont go into detail. Maybe i will add some comments.

ST-Link:
You should use a ST-Link V2 for programming the Motor Controllers and the Main Controller.
- ST-Link V2 https://www.ebay.de/sch/i.html?_nkw=st-link+v2&_sop=12
- Virtual COM Port Drivers https://www.st.com/en/development-tools/stsw-stm32102.html

STM32 ST-Link Utility:
From factory the Hoverboard Motorcontrollers are locked, so you have to unlock them by using the STM32 ST-Link Utility and a Flash Device called ST-Link V2. To use the ST-Link you have to install the Virtual COM Port Drivers.
- STM32 ST-Link Utility https://www.st.com/en/development-tools/stsw-link004.html
- How To Unlock https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/wiki/How-to-Unlock-MCU-Flash


## Electronics
I would say the electronics of my Bobbycar are pretty advanced, i can control the settings via bluetooth, i have Torque Shifting and a brake light. I would recommend you to not implement those features until it drives. Otherwise you could run into a few problems. These are the parts i used just so this instruction is complete. It would be far easier to take an arduino nano and two potentiometers and flash the arduino code you find in the repository here: https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/tree/master/Arduino/hoverserial.

Parts:
- 2x Hoverboards
- 2x 13s eBike Battery with BMS
- Potentiometers out of old XBox360 Controller
- STM32F4 Black Pill (STM32F1 Blue Pill should also work but with another pin configuration)
- Bluetooth HC05
- Display 1602 LCD
- Steering Sensor AS5600
- XL7015 DCDC Converter
- 1x N Mosfet (over 60V V_DS, whatever is lying around, like 110N8F6)
- 1x P Mosfet (over 60V V_SD, whatever is lying around, like IRF9530N)
- 2x Diode
- 2x 10k Resistor
- 1x 50k Resistor
- Port Expander PCF8574A

Schematic for Electronics: https://github.com/Kowitzki/Four-Wheeled-Bobbycar/blob/main/SchematicFourWheel.png


Accumulator
I used two ebike accumulators from aliexpress. They have more voltage than those from Hoverboards, which means a higher speed. Also they have more capacity. Till now they last far better than the original ones.
Two of them have a capacity of about 550Wh, which lasts for 40km at about 20kmh and much coasting without braking.

Links:
- The Schematic of the Hoverboard Controllers https://beta.ivc.no/wiki/images/d/d7/Hoverboard_schematic_full_reverse.pdf
- STM32F4 BlackPill https://docs.zephyrproject.org/latest/boards/arm/blackpill_f411ce/doc/index.html
- STM32F4 BlackPill https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0.html


## Mechanics
The provided CAD Model is built in Autodesk Inventor, but there you can also find a step export later.

### Frame
The Frame is built from 6mm and 21mm Plywood (Sperrholz), the kind with long fibres, not pressboard/particle board (Pressspan). Plywood is much more rigid.
There are dxf-files under "Mechanics" for lasercutting. Cutting by Hand is also possible, therefor you need an A3 printer and a jigsaw.
After cutting, 12mm parts are glued together out of 6mm pieces, then everything is pushed together and tightend by screws. Assembling the steering is not that easy, my bad.
If you want your wood to last longer in the rain i recommend you to paint it.

### Steering
The uprights are printed using a 0.8mm or 1mm nozzle, a layer hight of 0.3mm and 60-80% infill. There are two options for the steering, one for normal driving and one for drifting. The options have a different geometry.
Unfortunately the steering shaft has to be welded, but i think every formula student team has someone in their team who can weld. Stick welding is sufficient.
If you use normal structural steel like S235 the steering shaft can be bend easily, already happend twice to me. Maybe you consider getting a tougher steel for the shaft.



# Drift Tires
Yeah, you read it right, i built drift tires for this. Fortunately a pipe in Germany named "Kanalgrundrohr" has the right size for this and is made out of PVC. A slice of this can be glued to printed PLA parts, which are mounted to the hoverboard motors using an TPU adaptor. This has been built, but not been tested yet. I am confident that it will work.



# Formula-Student-Mini
Maybe we can establish a formula competition which is about small, extremely low cost and high powered vehicles. At my University we set up some rules. Some are for safety, some are to harmonize cars and some are for fun.
- Rules https://github.com/Kowitzki/Four-Wheel-Bobbycar/blob/main/Formula%20Student%20Mini%20-%20Rules%20V1.pdf

We have a Telegram Group for the Bobbycars, get in there, maybe someone will help you: https://t.me/FormulaStudentMini



# Known Problems
- The steering is not going back to forward if you let go, and there is much friction. I already thought of putting bearings in there. Also a judge in FSA told me a spring could handle the caster problem.
- The Display is not working on the I2C Line, if the steeringsensor and the port expander for the brake light are used.
- If you have a four wheel car with two MCs, do not drive in field weakening when one controller is off. It would get more voltage than it is supposed to.



# Other Instructions for Bobbycars
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
www.paypal.me/kowitzki/2.00
