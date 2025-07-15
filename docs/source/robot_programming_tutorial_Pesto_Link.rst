Robot Programming with PestoLink-Receive
=============================================

This tutorial will guide you through programming, uploading, and driving a robot. In this tutorial we will use the Arduino libraries `Alfredo-NoU3`_ and `PestoLink-Receive`_. We will also use the web app `PestoLink <https://pestol.ink/>`_ to control a robot wirelessly.

Setup
-----

| 1. **Get the Arduino IDE.** This lets us write and upload programs to our robot. Download it from the `Arduino website <https://www.arduino.cc/en/main/software>`_ and install it.

| 2. **Open the Library Manager** In the Arduino IDE, open the Library Manager by clicking the books icon on the left.

| 3. **Install the** `Alfredo-NoU3`_ **and** `PestoLink-Receive`_ **libraries.** Click *Filter your search...* and type **Alfredo-NoU3**, then click **INSTALL**. Do the same for the library **PestoLink_Receive**.

| 4. **Configure the Arduino IDE to upload to a NoU3.** Go to **Tools** > **Board** > **Boards Manager**. Search for "ESP32" and install the package "esp32 by Espressif Systems". Use the latest version.

| 5. **Decide what device you will use to drive your robot.** With a laptop, you can control your robot using the keyboard or a gamepad (for example an Xbox controller). With a smartphone, you can drive your robot with the touch screen.

| 6. **Pick a Web Browser.** PestoLink works best with Google Chrome on Windows, Android, MacOS, Linux, and ChromeOS. On iOS `Bluefy <https://apps.apple.com/us/app/bluefy-web-ble-browser/id1492822055>`_ is the best option.

| 7. **Bookmark PestoLink.** go to `https://pestol.ink/ <https://pestol.ink/>`_ and bookmark the page.

Basics
------

We're going to start by writing a simple program to test using PestoLink to control any motors or servos connected to your NoU3 via the ESP32's Bluetooth.

.. note:: For this section, you'll at least need Alfredo NoU3 ready to drive some motors. Follow our `Build Tutorial <https://youtu.be/rcdbkeYA1rs>`_ to set that up before continuing here.
   
.. seealso:: If you just need an example program to test motor and servo movement without an explanation, use the **motor party** example sketch included with Alfredo-NoU3.
   
When you open up the Arduino IDE, you'll see an empty program (a "sketch") that looks like this:

.. code-block:: cpp

    void setup() {

    }

    void loop() {

    }

These are two **functions**, or chunks of code. When your program runs on an ESP32, the ``setup`` function runs once when the program starts, then the ``loop`` function runs repeatedly. Right now, they're empty, but we'll add some code to them soon.

Before that, we need to make some changes at the start of the sketch, before the functions. First, we need to specify that we want to use the libraries we installed earlier, Alfredo-NoU3 and PestoLink-Receive. To do that, add the following lines to the top of the file.

.. code-block:: cpp

    #include <Alfredo_NoU3.h>
    #include <PestoLink-Receive.h>
   
These statements reference some code from a separate file that you want to have included in your final program.

Next, we'll declare the motors and servos we want to use.

.. code-block:: cpp

    NoU_Motor leftMotor(2);
    NoU_Servo armServo(3);

In the first line here, we say that there's a :cpp:class:`NoU_Motor` called ``leftMotor`` assigned to motor port 2. You can check which motor port your motor is plugged into by checking for the little labels where the wires plug in. For example, motor port 2 has labels that say **M2-** and **M2+** on the NoU3. Similarly, in the second line, we construct an object called ``armServo`` of type :cpp:class:`NoU_Servo` on servo port 3. The servo pins on the NoU3 are labeled **S1**, **S2**, and so on.

.. note:: :cpp:class:`NoU_Motor` is a **class** defined in the Alfredo-NoU3 library. A class is like a template that can be used to make an **object**, a specific instance of a class. In this case, the name we give to the specific instance of a :cpp:class:`NoU_Motor` we're creating is ``leftMotor``, but we're allowed to call it whatever we want (like ``rightMotor``, ``m``, or ``john``). Lastly, we put a 2 in parentheses at the end to say we want motor port 2. 

Your setup is likely not just one motor and one servo, so make sure you have one line for each motor and/or servo. Make sure each one has a different name and the correct motor or servo port number.

Altogether, our code so far looks something like this (with different motors and servos based on your setup):

.. code-block:: cpp

    #include <Alfredo_NoU3.h>
    #include <PestoLink-Receive.h>

    NoU_Motor leftMotor(2);
    NoU_Servo armServo(3);
    
    void setup() {

    }

    void loop() {

    }

Now, we're going to start adding code to the ``setup`` function. This code will run as soon as the program starts. Pretty much any time the ESP32 is on, the last program you uploaded to it will be running. You can restart the program by either cycling power to the ESP32 or by clicking the little **EN** button. We'll start by enabling PestoLink, and starting up the NoU3 motor ports.

.. code-block:: cpp

    void setup() {
        NoU3.begin();
        PestoLink.begin("Name me!"); // Change this name to the name of your robot before uploading! Also, this is a code comment.
    }

This will cause the ESP32 to start advertising itself as a Bluetooth peripheral when it's on. Make sure to give it a unique name or it'll conflict with the Bluetooth name of anyone else who followed this tutorial.
      
.. note:: It's standard to add a level of indentation any time you enter a set of curly braces, like inside of a function. It doesn't affect the program, but it makes it easier to read.

Next, let's set up controlling a motor remotely with PestoLink. Add the following code to the ``loop`` function.

.. code-block:: cpp

    void loop() {
        if (PestoLink.buttonHeld(0)) {
            leftMotor.set(1);
        }
        else {
            leftMotor.set(0);
        }
    }

This code will make ``leftMotor`` move while **Button 0** is held on PestoLink and stop when it's not held. Don't forget that your motors may be called something different and you may have more than one, so change the name and duplicate as needed.

.. note:: How did we know to set ``leftMotor`` to 1 to start it and 0 to turn it off? You can look at the :ref:`api-reference` to see specific details about all the functions in Alfredo-NoU3. Specifically, the :cpp:func:`NoU_Motor::set` function takes a number between -1 (full reverse) and 1 (full forward), including decimal numbers.
   
.. note:: `PestoLink`_ gives you many options to control your robot with buttons. For example, if you have a gamepad (like an Xbox controller) plugged into your laptop, you can press the **A Button** to trigger **Button 0**. In fact, every button on a gamepad has a corrisponding number you can use in `PestoLink`_. If you disable the Mobile Layout, you can see the number for each button.

Now, let's do something similar for our servos. Add the following code to the ``loop`` function for each servo you have.

.. code-block:: cpp

    if (PestoLink.buttonHeld(1)) {
        armServo.write(180);
    }
    else {
        armServo.write(0);
    }
   
This will make ``armServo`` move to its 180° position while **Button 1** is held and move back to its 0° position while it's released. Again, change the name and duplicate as needed for the servos in your setup.
   
That should complete our test sketch. Here's what the final code should look like (but with the number of motors and servos adapted for your setup):
   
.. code-block:: cpp

    #include <Alfredo_NoU3.h>
    #include <PestoLink-Receive.h>

    NoU_Motor leftMotor(2);
    NoU_Servo armServo(3);
    
    void setup() {
        NoU3.begin();
        PestoLink.begin("Name me!"); // Change this name before uploading!
    }

    void loop() {
        if (PestoLink.buttonHeld(0)) {
            leftMotor.set(1);
        }
        else {
            leftMotor.set(0);
        }

        if (PestoLink.buttonHeld(1)) {
            armServo.write(180);
        }
        else {
            armServo.write(0);
        }
    }
    
Uploading
*********
Now we need to upload our sketch to the robot.

1. **Connect to the board.** Connect the NoU3 to your computer using a USB-C cable.
2. **Select the NoU3 Board file.** Click the "Select Board" drop down near the top left of the Arduino IDE. Click Select other board an port. In the BOARDS section, search for and select **Alfredo NoU3**. In the COM section, ensure the COM port for your NoU3 is selected.
3. **Upload.** Clicking the **Upload** button (the arrow in the top left pointing right. If you are not sure, hiver over the button, it will say Upload).
   
Bluetooth Pairing
*****************
Now that we've uploaded that sketch, the ESP32 will advertise itself as a Bluetooth device as long as it's on.

**Press/click the CONNECT button in** `PestoLink`_. A pairing menu will appear, find and select the robot name you chose.

You should now be connected to your robot in PestoLink via Bluetooth. Press the buttons you assigned (this tutorial used the 0 and 1 buttons) and the motors and servos you have connected to your robot should move.
    
Robot Programming
-----------------

Now, we'll adapt the sketch from the previous section to drive a robot. We'll assume the robot is the one from the `Build Tutorial <https://youtu.be/rcdbkeYA1rs>`_, so it has a servo to raise and lower a basket and a drivetrain with two motors (technically, four motors using only two motor ports).

First, we'll change our list of motors and servos to match our robot.

.. code-block:: cpp

    NoU_Motor leftMotor(1);
    NoU_Motor rightMotor(2);
    NoU_Servo basketServo(1);
    
Now, we'll create a ``NoU_Drivetrain`` with our two motors. We'll use this to move the drivetrain motors as a group instead of individually setting the power that goes to each one.

.. code-block:: cpp

    NoU_Drivetrain drivetrain(&leftMotor, &rightMotor);
   
Next, we'll change the ``loop`` function to make the robot drive. This will be different depending on whether you want to control your robot with a keyboard or a gamepad.
   
On-Screen Joystick
******************

We'll use the On-Screen Joystick to drive the robot in this tutorial, but you can use a gamepad or the WASD Keys alternitively. The vertical axis (axis 0) will control the throttle of the robot and the horizontal axis (axis 1) will control the rotation.

.. code-block:: cpp

    void loop() {
        float rotation = 0;
        float throttle = 0;
        
        // Set the throttle of the robot based on what key is pressed
        rotation = -1 * PestoLink.getAxis(0);
        throttle =  1 * PestoLink.getAxis(1);
        
        // Make the robot drive
        drivetrain.arcadeDrive(throttle, rotation);
    }
    
Lastly, we'll make our servo move when we press **button 0**.
    
.. code-block:: cpp

    if (PestoLink.buttonHeld(0)) {
        basketServo.write(180);
    }
    else {
        basketServo.write(0);
    }
    
Our completed sketch will now let us drive the robot. Upload it to the NoU3, power on your robot, and connect to it with PestoLink on your computer.

.. code-block:: cpp

    #include <Alfredo_NoU3.h>
    #include <PestoLink-Receive.h>

    NoU_Motor leftMotor(1);
    NoU_Motor rightMotor(2);
    NoU_Servo basketServo(1);
	
    NoU_Drivetrain drivetrain(&leftMotor, &rightMotor);

    void setup() {
        NoU3.begin();
        PestoLink.begin("Name me!"); // Change this name before uploading!
    }

    void loop() {
        float rotation = 0;
        float throttle = 0;
        
        // Set the throttle of the robot based on what key is pressed
        rotation = -1 * PestoLink.getAxis(0);
        throttle =  1 * PestoLink.getAxis(1);
        
        // Make the robot drive
        drivetrain.arcadeDrive(throttle, rotation);

        // Control the servo
        if (PestoLink.buttonHeld(0)) {
            basketServo.write(180);
        }
        else {
            basketServo.write(0);
        }
    }

Troubleshooting
-----------------
Some USB-C cables are "power only", they cannot be used to upload code to a NoU3. If your PC does not find your NoU3, make sure your USB-C cable is known to be a data cable.

If you ever fail to upload code, and the IDE says "fatal error occured: No serial data recieved." then you may need to put your NoU3 into Boot Mode.

To put your NoU3 into Boot Mode, start by powering off the NoU3. Then press and hold the boot button. While holding the button, power on the NoU3. Once it has power you can let go. Now that it is in boot mode you should be able to upload code like normal. Once code is uploaded, when you reset/power cycle the NoU3, it will be fixed.

.. _Alfredo-NoU3: https://github.com/AlfredoSystems/Alfredo-NoU3/
.. _PestoLink-Receive: https://github.com/AlfredoSystems/PestoLink-Receive/