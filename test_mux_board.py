import pyvisa
rm = pyvisa.ResourceManager('@py')
print(rm.list_resources())
samd21 = rm.open_resource('USB0::51966::16384::123456::0::INSTR')
print(samd21.query("*IDN?"))
# CONNECT DAC OUTPUT TO ADC INPUT
print(samd21.query("SOURC1:VOLT:LEV?"))
samd21.write("SOURC1:VOLT:LEV 2.0")
print(samd21.query("SOURC1:VOLT:LEV?"))
print(samd21.query("SENS1:VOLT?"))
# CONNECT DAC OUTPUT TO MUX input 16, ADC INPUT TO MUX1 output
samd21.write("SOURC1:VOLT:LEV 2.0")
print(samd21.query("SENS1:VOLT?"))
print(samd21.query("MUX1:SEL?"))
samd21.write("MUX1:SEL 16")
print(samd21.query("MUX1:SEL?"))
print(samd21.query("MUX1:EN?"))
samd21.write("MUX1:EN 1")
print(samd21.query("SENS1:VOLT?"))
samd21.write("SOURC1:VOLT:LEV 1.2")
print(samd21.query("SENS1:VOLT?"))
samd21.write("*RST")

