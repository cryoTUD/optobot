# 03 LED
This ReadMe will discuss everything relating the added LED functionality. Important to note: this module is not required for the basic plunging setup. The LED module is divided in the following components:
- LED drivers
- LED sources
- Lens mount assembly

What the LED module offers is sample activation through LED light. The moment LED light is shone onto the biological samples, they respond to the light. Right after the LED, the sample is plunged into the ethane. By timing the plunging motion after the LED activation, one can capture how the samples are affected by the LED activation. This is because the plunging vitrifies the samples, so a snapshot can be created of the sample in its state right after being shone upon by an LED.

The LED light can be varied by using different wavelengths. In our setup, UV light and Green light were applied to the sample. These LED sources are also supplied in the BOM. If, for research, other LED sources are required, Thorlabs has a wide range of different-wavelength LED sources. So the sources mentioned in the BOM can be replaced with other sources. 

In the current setup, there are 4 UV light sources used, controlled by 4 identical LED drivers. These 4 LED sources are connected through a 4-to-1 fan-out cable into a single lens. A research project that was performed with the Optobot Plunger was to compare the sample at 4 different time points after LED activation. This was approached with the 4 UV LED sources, which through the fan out cable all were able to shine onto the sample. Through the fan-out cable and the lenses, these 4 sources created a 2x2 grid of LED spots on the sample. By altering the LED activation time of the 4 sources, on a single sample, 4 spots would be created with a different LED activation timing. By then plunging the entire sample right after, within the research, the development of the sample after LED activation could be captured at 4 different time points.

So, depending on the type of research that will be conducted, the LED module can be removed or modified. Within this repository, the entire LED module is documented as used in the setup. In the Optobot setup, 4 LED drivers were used to control the 4 UV LED sources. In the case when it was switched to only the single green LED source, one of the LED drivers was disconnected from the UV LED and connected to the green LED. The number of LED drivers therefore depends on the number of LED sources that need to be controlled individually.

## Lens mount assembly
Within the LED module, there is only 1 assembly that exists besides the individual LED parts, and that is the Lens mount assembly.
