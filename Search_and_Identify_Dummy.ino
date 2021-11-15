enter search mode
calibrate analog voltage swing of phototransistor and receiver over 1s.

phototransistor:
if value > 40, start average value measurements over 60us.
  if average value is between 230 and 255,
    set pt1_high to true.
  if average value is between 110 from 130,
    set pt1_high to false.
  
wait 10us

receiver:
start average value measurements over 60us.
  if average value is between 230 and 255,
    set r1_high to true.
  if average value is between 0 and 20,
    set_r1_high to false.

repeat loop after 530 us for pt2_high and r2_high.
