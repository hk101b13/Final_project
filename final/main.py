THRESHOLD = (0,100)
BINARY_VISIBLE = True
import pyb, sensor, image, time, math
enable_lens_corr = False # turn on for straighter lines...
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # grayscale is faster
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)

def degrees(radians):
   return (180 * radians) / math.pi

uart = pyb.UART(3,9600,timeout_char=1000)
uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)


while(True):
   clock.tick()
   img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()
   if enable_lens_corr: img.lens_corr(1.8) # for 2.8mm lens...
   line = img.get_regression([(180,255) if BINARY_VISIBLE else THRESHOLD],roi = (20,0,120,50), robust = True)
   if (line): img.draw_line(line.line(),color =127)



   if(170< line.theta()<=180 or 0<=line.theta()<10):
     dir = 'f'
   else:
     if(line.rho()<0):
       dir = "l"
     elif(line.rho()>0):
       dir = "r"
   print(line.theta())
   print_arg=(dir)
   print(dir)

   uart.write(("%s" % print_arg).encode())
   img2 = sensor.snapshot()
   if (img2.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y)): # defaults to TAG36H11
      dir = "s"
      print_arg=(dir)
      uart.write(("%s" % print_arg).encode())
      break

while(True):
   clock.tick()
   img = sensor.snapshot()
   for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
      img.draw_rectangle(tag.rect(), color = (255, 0, 0))
      img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
      # The conversion is nearly 6.2cm to 1 -> translation
      #print_arg(tag.y_rotation())
      #print(tag.z_rotation())
      if(3.1<tag.z_rotation()<3.2):
        dir = "s"
      else:
        dir = "k"
   print_arg=(dir)
   print(dir)
      # Translation units are unknown. Rotation units are in degrees.
   uart.write(("%s" % print_arg).encode())
   #uart.write(("FPS %f\r\n" % clock.fps()).encode())

