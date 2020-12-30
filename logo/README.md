To generate your own logo data:

1. create a monochrome PNG image with 64x64 px
2. convert to C code array using [image2cpp](https://github.com/javl/image2cpp) (use the [online service](https://javl.github.io/image2cpp/))

   You need the following settings:

   -  possibly invert image color (black background looks best on OLED display)
   -  code output format: *plain bytes*
   -  draw mode: *Vertical - 1 bit per pixel*
