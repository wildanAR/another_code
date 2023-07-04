# import EAN13 from barcode module
from barcode import EAN13
from barcode import EAN8

# import ImageWriter to generate an image file
from barcode.writer import ImageWriter

# Make sure to pass the number as string
number = '12345678'

# Now, let's create an object of EAN13 class and
# pass the number with the ImageWriter() as the
# writer
my_code = EAN8(number, writer=ImageWriter())

# Our barcode is ready. Let's save it.
my_code.save("new_code1")
