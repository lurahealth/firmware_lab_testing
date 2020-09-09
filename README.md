This application is to be used by Lura Health for laboratory test purposes. The applicatiion utilizes Nordic UART Service, advertises itself ar "Lura Health", and continually streams data until a connection is broken. When connection is terminated, the application will restart advertising.

This application was built upon the UART Example. To build and run, place all files into SDK_ROOT/examples/ble_peripheral/_folder_name_, where _folder_name_ is the folder containing all files within this repository. This application is stable when used with nRF5_SDK_15.2.0.

**Packet format:**

        [*calibrated_pH*, *calibrated_Na_mmol_L*, *calibrated_K_mmol_L*,  
         *raw_pH_mV*, *raw_Na_mV*, *raw_K_mV*, *temperature_celsius*, *battery_mV*]

         Where every field is 4 human readable ASCII characters representing the numerical value
        
**How to calibrate:**

        - Before calibration occurs, the first, second and third fields of  
          the packet will contain "0000" in place of a real value
        - Begin calibration by sending "STARTCAL_AX" packet, where X can be 1, 2 
          or 3, corresponding to 1 point, 2 point or 3 point calibration and A can
          be P (for pH), N (for Na+), or K (for K+). 2 point or 3 point 
          calibration should be performed before a 1 point calibration is performed.
                - The peripheral (this application) will return a "CALBEGIN" 
                  confirmation packet
        - For X calibration points indicated within "STARTCALX" packet, send 
          packets following the format  "PTN_AB.C" where N is every value from
          1 to X and AB.C is a mmol/L or pH value to be referenced for the calibration 
          reading. AB.C can also have format A.BC - 3 digits with one decimal
                - The peripheral will return a "PTXCONF" confirmation packet
        - Once X "PTN_AB.C" packets have been sent, the peripheral will run a 
          linear regression on the set of points (or adjust offset for a 1 
          point calibration)
        - Calibrated analyte values will now be sent in the data packets

        Example of 3 point calibration for pH:

                1. Central:  "STARTCAL_P3"
                   Response: "CALBEGIN"
                2. Central:  "PT1_31.2"
                   Response: "PT1CONF"
                3. Central:  "PT2_10.0"
                   Response: "PT2CONF"
                4. Central:  "PT3_3.20"
                   Response: "PT3CONF"
                5. Data packets will now include calibrated pH data
                
         Example of 2 point calibration for Na+:
         
                1. Central:  "STARTCAL_N2"
                   Response: "CALBEGIN"
                2. Central:  "PT1_31.2"
                   Response: "PT1CONF"
                3. Central:  "PT2_10.0"
                   Response: "PT2CONF"
                4. Data packets will now include calibrated pH data
