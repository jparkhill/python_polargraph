
Some simple code to drive...
# The simplest plotter possible?
- minimal dependencies.
- trivial geometry.
- basically just imperative python.

## Usage
- to test:
    python3 -i plotter.py

## Hardware
- adafruit stepper kit
<iframe style="width:120px;height:240px;" marginwidth="0" marginheight="0" scrolling="no" frameborder="0" src="//ws-na.amazon-adsystem.com/widgets/q?ServiceVersion=20070822&OneJS=1&Operation=GetAdHtml&MarketPlace=US&source=ac&ref=tf_til&ad_type=product_link&tracking_id=johnparkhill-20&marketplace=amazon&region=US&placement=B00TIY5JM8&asins=B00TIY5JM8&linkId=69e22ae7bcafdb9ec58d0c3a8bdfe4e4&show_border=false&link_opens_in_new_window=false&price_color=333333&title_color=0066C0&bg_color=FFFFFF">
    </iframe>
- Nema17 steppers (x2)
- A RaspberryPi (any sort)
- some wire, jumpers,
- A board.
(optional) some 3d printed brackets.

## Video Explanation.

## How to setup the PI

- enable I2C (req'd, "c.f. raspi-config")
- Connect it to the internet.
- install all desired adafruit libraries,
    and imaging if you want to make line art
- install this package.
