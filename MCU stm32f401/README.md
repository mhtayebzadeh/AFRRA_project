# Enable Float Formatting in your STM32 Project (Crucial)

If you are seeing missing floating-point values (empty spaces where numbers should be) over serial, it means the compiler has disabled floating-point support for sprintf to save memory.

**If using STM32CubeIDE, follow these steps to enable it:**
1. Right-click your project in the Project Explorer -> **Properties**.
2. Go to **C/C++ Build** -> **Settings**.
3. Under **Tool Settings**, go to **MCU Settings**.
4. Check the box for: **Use float with printf from newlib-nano (-u _printf_float)**.
5. Click **Apply and Close**, then rebuild your project and re-flash the MCU.
