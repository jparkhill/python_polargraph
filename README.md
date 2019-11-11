
Some simple code to drive...
# The simplest plotter possible?
- minimal dependencies. trivial geometry. basically just imperative python.


## Example Input path (made in the ipython notebook)
![Ideal path](https://github.com/jparkhill/python_polargraph/blob/master/images/input.png)
## And the resulting output
![What I got](https://github.com/jparkhill/python_polargraph/blob/master/images/output.jpg)

```
  python3 -i plotter.py
  >>> pl.draw_circle(pl.center)
  >>> pl.choose_file()
```

## Mandatory Hardware
- adafruit stepper kit (~20$)
- Nema17 steppers (x2) 300ma, 12V (these suckers are barely up to the task but all the kit can drive.) (~10$)
- 2 ordinary SG 90 servos. (~5$)
- a 12V power supply. (~5$)
- A RaspberryPi (any sort) (~35$)
## Optional/improvised stuff
- A PiTFT
- a wireless keyboard
- some wire, jumpers,
- A whiteboard
- fine-tip sharpies
- Some roller blind cord.
- M3,M4 bolts and nuts (get a set on 'zon, don't go to Home Depot)
- 3d printed components (linked)

## Hardware Commentary
- The adafruit ppl are pretty smart to use a cheap LED PWM controller to drive the Toshiba stepper controller in the hat. But the driver software was a bit rushed, like everything adafruit. If you use their stock packages, you can't use servo pin 15 for the lifters while using the steppers because the frequency of the PWM generator needs to be changed. Also the stepper library defaults to fully energizing both coils too often to try to maximize power/torque. In my setup this caused the steppers to overheat, lose steps (due to thermal resistance presumably) and fail because about a half an Amp of power was being dumped into each stepper. I hacked up about 3 separate repos worth of adafruit code to make one plotter_kit.py driver that works for this specific application. I commented out the double stepping etc. YMMV.

## How to install/setup the PI

- Connect the stepper hat to the pi's I2C pins (SDA, SCL) and 3.3,5V and GND pins.
- Power the stepper hat as well, wire up your steppers.
- write the servos data lines to the empty connector 15 on the stepper hat, and power them with the 5.5V and GND lines on the stepper board.
- Starting with blank raspbian, connect to wifi and clone this repo.
- enable I2C (req'd, "c.f. raspi-config")
- Connect it to the internet.
- sudo pip3 install -e .

## Debug tips
- It's easy to check the coil voltages with a multimeter. Don't exceed the 12 they are rated for.
- If the resistance of your stepper is too low, the stepper hat will blink green, and stop driving.
- Counterweights should be the smallest weight possible that rests the gondola about 13cm from the inter-cog line.

## Useful 3d Prints.
- The cogs I used: https://www.thingiverse.com/thing:569308
- Brackets I used: https://www.thingiverse.com/thing:3169670 (I recommend metal instead for heat-sink)
- A gondola I used: https://www.thingiverse.com/thing:478346

## Bonus
Some random python line-artifiers I wrote while on planes to make stuff like:
![A sad dood](https://github.com/jparkhill/python_polargraph/blob/master/images/StalkerII.png)
