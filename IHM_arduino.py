import serial, time, sys
import matplotlib.pyplot as plt
import numpy as np
#from drawnow import *
from scipy.fftpack import fft, fftfreq
import math
import PySimpleGUI as sg
import logging
logging.getLogger().setLevel(logging.CRITICAL)
from datetime import datetime


def LEDIndicator(key=None, radius=30):

    return sg.Graph(canvas_size=(radius, radius),
             graph_bottom_left=(-radius, -radius),
             graph_top_right=(radius, radius),
             pad=(0, 0), key=key)

def SetLED(window, key, color):
    graph = window[key]
    graph.erase()
    graph.draw_circle((0, 0), 20, fill_color=color, line_color=color)

def Map(value, fromLow, fromHigh, toLow, toHigh):
    return ((value-fromLow)*(toHigh-toLow))/(fromHigh-fromLow)+toLow

def Updating_output():

    Vmax_Master_Output.Update('{0:f}'.format(Vmax_master_data))
    Vrms_Master_Output.Update('{0:f}'.format(Vrms_master_data))
    Vmax_Slave_Output.Update('{0:f}'.format(Vmax_slave_data))
    Vrms_Slave_Output.Update('{0:f}'.format(Vrms_slave_data))

    Delta_Vrms_Output.Update('{0:f}'.format(Delta_Vrms))

    SetLED(window, 'ΔVrms' , 'green' if abs(Delta_Vrms) < Threshold_Delta_Vrms else 'red')
    SetLED(window, 'Vmax_M', 'green' )#if Delta_Vrms > 0 else 'red')
    SetLED(window, 'Vmax_S', 'green' )#if Delta_Vrms > 0 else 'red')
    SetLED(window, 'Temperature', 'green')# if Delta_Vrms > 0 else 'red')

    H0_Output.Update('{0:f}'.format(results[0]))
    H1_Output.Update('{0:f}'.format(results[1]))
    H2_Output.Update('{0:f}'.format(results[2]))
    H3_Output.Update('{0:f}'.format(results[3]))
    H4_Output.Update('{0:f}'.format(results[4]))
    H5_Output.Update('{0:f}'.format(results[5]))
    H6_Output.Update('{0:f}'.format(results[6]))
    H7_Output.Update('{0:f}'.format(results[7]))

    force.set_ydata(force_data)
    fft_m.set_ydata(FFT_master)
    fft_s.set_ydata(FFT_slave)
    master.set_ydata(master_data)
    slave.set_ydata(slave_data)

    for harm in range(0, len(harmonics)):
        eval('h'+str(harm)).set_data(x_harm,harmonics[harm])


    ax[1,0].relim()        # Recalculate limits
    ax[1,0].autoscale_view(True,True,True) #Autoscale
    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    time.sleep(0.0001)

def Readjust_data(master_data,slave_data):

    master_data = Remove_offset(master_data)
    slave_data = Remove_offset(slave_data)

    master_data = Map(master_data,-max_raw_value/2,max_raw_value/2,-11,+11)
    slave_data = Map(slave_data,-max_raw_value/2,max_raw_value/2,-11,+11)
    return master_data,slave_data

def Remove_offset(data):
    data = np.array(data)-np.mean(data)
    return data

def goertzel(samples, t, freqs):
    channel
    total_point = len(samples)
    t = np.linspace(0, total_point/Fech, total_point)
    results = []
    w_real, w_imag = [], []

    for f in freqs:
        '''  Cosine and sin computation  '''
        w_real = np.cos(2.0 * math.pi * f* t)
        w_imag = np.sin(2.0 * math.pi * f *t )

        '''  Computation  '''
        x  = np.multiply(samples, w_real)
        y  = np.multiply(samples, w_imag)

        '''  Average computaion  '''
        xavr = np.average(x)
        yavr = np.average(y)

        '''  Module computaion  '''
        module = round(2 * np.sqrt(xavr**2 + yavr**2),2)
        results.append(module)
    return results

def splitting(f_m_s_data, master_data, slave_data):

    force_data = np.array((f_m_s_data[0:(len(f_m_s_data)//3)]))
    master_data = (f_m_s_data[(len(f_m_s_data)//3):(2*len(f_m_s_data)//3)])
    slave_data  = (f_m_s_data[(2*len(f_m_s_data)//3) :len(f_m_s_data)])
    return force_data, master_data, slave_data

def FFT(master_data,slave_data):

    yf2 = fft(master_data)
    yf3 = fft(slave_data)
    FFT_master= (2.0/Sample) * np.abs(yf2[0:Sample//2])
    FFT_slave= (2.0/Sample) * np.abs(yf3[0:Sample//2])
    return FFT_master, FFT_slave

def MAX(master_data,slave_data):
    return max(master_data), max(slave_data)

def RMS(Vmax_master_data, Vmax_slave_data):
    Vrms_master_data = (1/(2*np.sqrt(2)))*Vmax_master_data
    Vrms_slave_data  = (1/(2*np.sqrt(2)))*Vmax_slave_data
    return Vrms_master_data,Vrms_slave_data

def Acquisition():
    string='4'                                   # code corresponding to the acquisition
    f_m_s_data = []
    arduino.write(string.encode('utf-8'))        # send acquisition code
    time.sleep(0.001)                            # synchronization time
    buf_merged_v_m_s = arduino.read(len_buf*6)   # acquisition of the 3 signals

    j=0
    for i in range (0,len_buf*3 ):
        f_m_s_data.append(buf_merged_v_m_s[j]+0x100*buf_merged_v_m_s[j+1]) # Conversion Hex to Dec
        j+=2
    f_m_s_data = np.array(f_m_s_data)

    return f_m_s_data

def event_handler(event, values, mgn, f, statue_supply, statue_algo):

  # ------------  Button interaction -------------------

      # ----- Exit
    if event is None or event == 'Exit':

        Send_command(0,0)
        time.sleep(0.1)
        Send_command(2,0)
        window.close()
        plt.close()
        sys.exit()

      # ----- Supply
    if event == '_SUPPLY_':      # Supplying Master & Slave

        statue_supply = not statue_supply
        window.Element('_SUPPLY_').Update(image_data=(toggle_btn_on, toggle_btn_off)[statue_supply])

        if statue_supply == True :
            print(datetime.now(),"---> Supply OFF..")
            mgn = 0
            Send_command(2,mgn)
            time.sleep(0.1)

            if statue_algo == False :
                Send_command(0,0)
                window.Element('_ENABLE_ALGO_').Update(image_data=(toggle_btn_on, toggle_btn_off)[not statue_algo])

        if statue_supply == False :
            print(datetime.now(),"---> Supply ON..")
            for i in range (0,mgn):
                Send_command(2,i)
                time.sleep(0.4)

      # ----- Frequency
    if event == 'Apply F':       # Modifying master frequency

        try :

            f = int(values[0])
            print(datetime.now(),"---> Frequency entered :", f)
            if statue_supply == False :
                Send_command(3,f)

        except ValueError :
            print ("Error : invalid value, submit an integer ")
            pass

      # ----- Magnitude
    if event == 'Apply M':       # Modifying master magnitude

        try :

            mgn = int(values[1])

            if mgn > max_voltage_accepted : mgn = max_voltage_accepted
            if mgn <  0: mgn =  0

            print(datetime.now(),"---> Magnitude entered :", mgn)
            if statue_supply == False :
                Send_command(2,mgn)
        except ValueError :
            print ("Error : invalid value, submit an integer ")
            pass

      # ----- Reset
    if event == 'RESET Param':   # Reset value by default
        print(datetime.now(),"---> Reseting .. F:57Hz M:5V")
        f, mgn = 57, 5
        Send_command(3,f)
        time.sleep(0.1)
        Send_command(2,mgn)

      # ----- RepCon Gain
    if event == 'Apply Gain':  # Modifying RepCon Gain
             gain = float(values[2])*100
             print(datetime.now(),"---> Gain entered :", gain/100)
             Send_command(5,int(gain))

      # ----- Algorithm
    if event == '_ENABLE_ALGO_': # Enable RepCon Algorithm

        statue_algo = not statue_algo
        window.Element('_ENABLE_ALGO_').Update(image_data=(toggle_btn_on, toggle_btn_off)[statue_algo])

        if statue_algo == True :
            Send_command(0,0)
            print(datetime.now(),"---> RepCon disabled")

        if statue_algo == False :
            print(datetime.now(),"---> RepCon enabled")
            Send_command(1,0)


    return statue_supply, statue_algo, mgn, f

def Send_command(command,parameter):
    string='{0:d}{1:d}'.format((command),(parameter))
    arduino.write(string.encode('utf-8'))





Sample                = 1024         # Nbr of sample of each signal (force, master, slave)
len_buf               = Sample       # Length of buffer
Fech                  = 10032        # Sampling Frequency [Hz]
Tech                  = 1/Fech       # Samplng Time [s]
max_raw_value         = 4096         # Raw threshlod value not to be exceeded
TAB_FREQ              = []           # Observed frequency
freqs                 = TAB_FREQ
observed_frequency    = 57           # for goertzel

gain_aop = 10
gain_bop = 4
max_voltage_accepted = 5 #[V]

Threshold_Delta_Vrms  = 0.3 / gain_aop / gain_bop # Volt
Threshold_Vmax_M      = 10                        # Volt
Threshold_Vmax_S      = 10                        # Volt
Threshold_Temperature = 0.1                       # Volt

magnitude            = 0             # Master Param
frequency            = 0

# We have to initialize the data for the plotting

xf          = fftfreq(Sample, Tech)[:Sample//2]  # xaxis for FFT
force_data  = np.zeros(Sample)                   # Force signal Array
master_data = np.zeros(Sample)                   # Master signal Array
slave_data  = np.zeros(Sample)                   # Slave signal Array
FFT_master  = np.zeros(Sample//2)                # FFT master Array
FFT_slave   = np.zeros(Sample//2)                # FFT slave Array
results     = [0,0,0,0,0,0,0]                    # tab that store goertzel magnitude

# For time axis by assuming there are no jitter
   # elapsed_time = Sample * 1/Fech
   # t = np.arange(0, elapsed_time, 1/Fech)
for i in range(1, 9):
    TAB_FREQ.append(i*observed_frequency)

# ---- For Goertzel ---------------

harmonics     = [[0], # Fundamental
                 [0], # First Harmonic
                 [0], # Second Harmonic
                 [0], # Third Harmonic
                 [0], # Fourth Harmonic
                 [0], # Fivth Harmonic
                 [0], # Sixth Harmonic
                 [0]] # Seventh Harmonic
nbr_harmonics = 7
data          = 0
x_harm        = [0]
nbr_data      = 0


# ---------------------------- PLOTITNG --------------------------------------

# Parameter of the Plotting
plt.close('all')
fig, ax = plt.subplots(2,2)
fig.set_facecolor('tan') # Background color

fig.subplots_adjust(top=0.950,
                    bottom=0.055,
                    left=0.050,
                    right=0.99,
                    hspace=0.225,
                    wspace=0.085)

# Graph for vibration
force, = ax[0,0].plot(force_data,'g',label='Vibration',linewidth=0.6)
ax[0,0].set_title("Mesure Vibration")
ax[0,0].set(ylim=(0,3))

# Graph for master and slave signal
master, = ax[0,1].plot(master_data,'b',label='Master',linewidth=0.8)
slave, = ax[0,1].plot(slave_data,'r',label='Slave',linewidth=0.8)
ax[0,1].set_title("Signaux appliquées aux pistons")
ax[0,1].set(ylim=(-15,15))

# Graph for the sigal piston FFT
fft_m, = ax[1,1].plot(xf, FFT_master, 'b-',label='Master')
fft_s, = ax[1,1].plot(xf, FFT_slave, 'r--',label='Slave')
ax[1,1].set_title("FFT master & slave")
ax[1,1].set(xlim=(10,400))
ax[1,1].set(ylim=(0,10))

# Graph the harmonics
h0, = ax[1,0].plot(x_harm, harmonics[0],label='H0',linewidth=1.5)
h1, = ax[1,0].plot(x_harm, harmonics[1],label='H1',linewidth=1.5)
h2, = ax[1,0].plot(x_harm, harmonics[2],label='H2',linewidth=1.5)
h3, = ax[1,0].plot(x_harm, harmonics[3],label='H3',linewidth=1.5)
h4, = ax[1,0].plot(x_harm, harmonics[4],label='H4',linewidth=1.5)
h5, = ax[1,0].plot(x_harm, harmonics[5],label='H5',linewidth=1.5)
h6, = ax[1,0].plot(x_harm, harmonics[6],label='H6',linewidth=1.5)
h7, = ax[1,0].plot(x_harm, harmonics[7],label='H7',linewidth=1.5)

ax[1,0].set_title("Force Goertzel")
ax[1,0].set(ylim=(0,3))

for i in range (0,2):
    for j in range (0,2):
        ax[i,j].grid(which='minor',color='lightgray',linestyle='--')
        ax[i,j].minorticks_on()
        ax[i,j].grid(which='major',color='white')
        ax[i,j].legend()
        ax[i,j].set_facecolor('antiquewhite')

# ----------------------- GUI INTERFARCE --------------------------------------

toggle_btn_off = b'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'
toggle_btn_on =  b'iVBORw0KGgoAAAANSUhEUgAAAGQAAAAoCAYAAAAIeF9DAAARfUlEQVRoge1bCZRVxZn+qure+/q91zuNNNKAtKC0LYhs3R1iZHSI64iQObNkMjJk1KiJyXjc0cQzZkRwGTPOmaAmxlGcmUQnbjEGUVGC2tggGDZFBTEN3ey9vvXeWzXnr7u893oBkjOBKKlDcW9X1a137//Vv9ZfbNmyZTjSwhiDEAKGYVSYpnmOZVkzTdM8zTTNU4UQxYyxMhpzHJYupVSvUmqr67pbbNteadv2a7Ztd2SzWTiOA9d1oZQ6LGWOCJAACMuyzisqKroqGo1eYFlWxDRN3c4512OCejwWInZQpZQEQMa27WXZbHZJKpVank6nFYFzOGAOCwgR2zTNplgs9m/FxcXTioqKEABxvBL/SAsRngCwbXtNOp3+zpSLJzf3ffS5Jc8X/G0cam7DMIqKioruLy4uvjoej7NIJBICcbDnIN78cBXW71qH7d3bsTvZjoRMwpE2wIirjg0RjlbRi1wBBjcR5zFUx4ajtrQWZ46YjC+Mm4Gq0ipNJ8MwiGbTTNN8a+PyTUsSicT1jXMa0oO95oAc4k80MhqNvlBWVjYpHo9rrqD2dZ+sw9I1j6Nl/2qoGCCiDMzgYBYD49BghGh8XlEJRA5d6Z8EVFZBORJuSgEJhYahTfj7afMweczkvMcUcct7iUTikvr6+ta+0xIWAwJimmZdLBZ7uby8fGQsFtMo7zq4C/e+cg9aupphlBngcQ5OIFAVXvXA6DPZ5wkUIr4rAenfEyDBvfTulaMgHQWVVHC6HTSUN+GGP78JNUNqvCmUIiXfmkwmz6urq3s/f/oBARFC1MTj8eaKigq6ajCW/eZXuKd5EbKlGRjlBngRAzO5xxG8z0v7AAyKw2cNH180wQEmV07B2dUzcWbVFIwqHY2ySJnu68p04dOuHVi/Zx3eaF2BtXvXQkFCOYDb48LqieDGxptxwaQLw2kdx9mZSCSa6urqdgZt/QDhnBfFYjECY1JxcbEWU4+8/jAe+/DHME8wYZSIkCMKgOgLwueFKRTAJMPsmjm4YvxVGFUyyvs2LbF8iRCIL7+dLjs6d+DhdUvw7LZnoBiJMQnnoIP5p1yOK//sG+H0JL56e3ub6uvrtU4hLEKlTvrBNM37iouLJwWc8ejKH+Oxjx+FVW1BlAgtosDzCJ4PxEAgfJa5RAEnWiNw39QHcPqQCfqltdXkSCSSCWTSaUgyYcn4IZegqAiaboJjVNloLDxnMf667qu47pVvY5e7E2aVicc+ehScMVw+80r9E4ZhEK3vA/At+BiEHGIYRmNJScnblZWVjPTGyxuW4Z9Xf0+DYZQKMLM/GP2AGOy+X+cfdyElPbVsKu6f/gNURCr0uyaTSXR2duqrOsTXEO3Ky8v1lQZ1JA/i2hevwbsH10K5gL3fxh1Nd+L8My7wcFdKJZPJGePGjWt+9dVXPcHDGGOWZT1YXFysTdu2g21Y3Hy3FlPEGQVgMNYfDNa35hpyDiM+E5Wo3VTRhIdm/AjlVrn2I3bv3o329nakUin9LZyR/mQFzjCtfMY50qkU2ne362dcx0V5tAI/mfMEmqq+qEkiKgwsfvtu7DqwCwHtI5HIA3RvWZYHiBDiy0VFRdrpIz/jnlcWwy7Nap1RIKYCwvJBwAhByBG/P1h/xBXA6Oho3DvtARgQsG0HbW3tSCZT4AQAzweDhyBQG3iwSD2Akqkk2tva4WQdGNzAgxf9O0Zbo8EFQzaWweLli0KuEkI0bNu2bRbRn/viisIhWom/t2N9aNqyPjpjUK5AHhfwvHb+2QKEKYbvT1iIGI/BcST27dsL13U8MBgPweB5HOFd6W+h+7kPEFXHdbBn7x44rouoGcXds+4FyzDwIo6Wjmas274u4BKi/TWEAeecVViWdWEkYsEwBJauecLzM6LeD/VV4H3VwoT4GVgw7nZsvPgDr17k1VtOuh315gQoV/lWCXDr2O9i44Uf6HrL6Nshs7k+Kj9r+LnuWzFzFWRKes8eraKAi4ddgtPK66GURGdXpw8GL6gBR/S9Emhhf95VShddHR06vjVh+ARcMma29llEXODJtY+HksQwBGFQwTkX51qWZZmmhY7eTryzvxk8xrWfEZq2g+iM2SfMxf+c8xS+Ov5r/aj2d/Vfw09nPY1LSudoR8nXYGH/nHFzUS8nQNoyN2fQTcrvgANlq6PHIS4wr3a+Jlw6nUY2kwFjwhNPeaAInzOED4B3ZXmgsQI9Q5yTzmaQTmf03P/YcCVUGtp1WL2nGQd7OnwJwwmDc7kQ4ktBsPDNraugogCPHMKCYjnOuKvh7sMu34VnL0K9mgDpFOCBmBXD9WfeCJlU2qop4EByetN57X/oCoZJpZNRUzQSUklPeXMGoQEQ+toXGOYT3yO8yOMUkQcU1zpDcKHnpLlHVYzE5KopmkukCaza+uvwswkLAuR00u4EyLq2dV5symT9uaMAGIYrx14VNm1u3YQrHr8ctYtH4eT7R+PKn16Bzbs2hf3fGH81ZMItEE9UGsY0YHblXMBWA0ZcjlalldJU+QVNMOlKuFLqlU2rmAt/pecTXARXGuMBE4BGY3QANtyW8MAjn4XmllLhi6PO0iEWbgJrW9eGlhphwTnnY4P9jO0d27yQiBjEys5rbhjeqK879u3AxUsvxBvdr8EabsIaYWEVW4mvvHYpNrdv1mOaxjRB9voxIL88t/ZZfXP9jBvg9rr6BY9ZkcDpJRM0sRzb8QnsrWweXj1OITA05wTcQhwkhC/GvH4CQfgACh8w4iLbsbXYmnjiRB1WodXwScf2vEXITua0yxdsMu1Ot4MZrD8gff6cEJ+ImBnT98RyIs5hVAkYFYY2CMiRNCoNvHdgvR4Ti8QwMXpGASBL1z+BfT37MLRkKG4bf4dW4seqkCitiY7UxCIuITHFfTACEcR9YueLKw2CyOkW4hjBcyB4QOXaaH7y9kdVjgZ8g6U92Z7zZTgvJ0BKg4akm/ydHeruTDd4lOtKYAY6hpsMWxKbw3G1JWMLAGECeHrTU/p+7sSvoJ5P7CfSjlqRCnEjpsGAvykXiqVAmefpDtGnzauij0Um+t0TaQiUkkiJJxGUQoponuOQUp7vbarfgyKlRaXa9xho97C+4vTwftuBjwq1Omd48KMHsK93n+ag6yffqEMLx6SQESHJiJDeShV9iRuII5EHggg5RlejcHzQJ/KAIVGmuZA4Rfr7KAqFHr9SqjvYC46J2BGt0o29G5C0PWTPn3CBP3nhg/RDM6pn6PtkJon1nev7+TLEUQ+sv1/fk4IfUznmGCHihdClv2C0qBKFYGjlzVjhqmf9uSGnW3JmsAZSeFYSgd6Z6PJ+VAExEQ3fgbDgfsaEbhgeG6FZqZ9DNgBIq3d628NDS4fi2Yt/gdkVcz02lApfKpuJn037X4wuPUmP2di60RNnffZOiLNe6HwOm/d6oo1M4WNSGNCa+K1nBSnlE1uEK531UeqBWat1hfBM2wAAFoq6PCNAr36hudBVEjv2f+J9pVSojg7PTw7p5FLKj4NMiNqyWij7EB5y0MyARz58KGyuP7EeC2cuwqa/2Ko97f9oWoLThtSH/YtXLNKbWgX6KdhGEMB/fbT02AARFM6wqWOj9tBdx4Eg38E3ebnvhwiWrz9EKNY8P0XkiTkRWmnM7w84xXFtSFdhQ+t7Hi2kwpiK2vA1lFLbSGRtIkBIrk0bNU3vCWsPWYajCkS/R0iFjakNWLDilsN+681P3YgNqfUQxQIQhX3eljTDCx3PoaX1nf59R6lSWX2wWfsfru8vhA5eYLaKfEXPwvAJ83WDNnEDMISvX4QIn9W6Qy98ibe2v6mlA+WDTB05NeQQKeVm4pBfU74QPXDWqWeBpQCZUWFWRSEQuS1NmvC5jmfxV8/8JZ58p/8KX7rqCcx9ZA5+3vY0jAqh9+ALOSRHbZrrX7fQPs0xQoQpbOrdgJ09rZoOyXRa6wvB8j10plc744Gz6HEN90MnIvTchecMEucwFoou7alLhU/3/xbv7f6N53DbDGefdnb4yVLKlez111+vKCkp2V1VVWXRtu21//1NtDirYZ5ggFs8t6oHimfBQ1mlXLgJ6QUEHS/+pL3cGIco5uAxoc1g6nO6XDhdju43hxge5zAvOYD2n50OFzIrdTv1kzn9By86VCMxK/ZlXFd/k/60srIyUDg897GqMN4WEkLljcj/P9eazqTR1ekp8oW//Be8tONFzTXTKxvx0PyHPQtXqWxvb281iSxKd3wpk8lodp3f+HVNMEmiS+ZFYwfJtiP3nxPxqgxY1SYiNRYiIyzttZtDDW/r1/T0Byl2USpgDaM+s4DYBBCNNYeZ+nkCQ4f/j0bx3+2VjuXYevB9zSVdXV36Gsas8i0nFlhcOasrNy4/5sW8uTq9ubbs2oKXPvylTpuSWRfzm+aH7oLruoRBh6aIbdsPEUvZto3JtVPQVDlDp7BQrlGQ5hJi0kd0wVfMRDweF7rS6qbwMnGYDuHniTwCh/pELC9Eo/JA0Vwl9J6BflbhqFT9LiZwz/t3I5FN6D2MvXv3Qfoh+HxdEYixcKcw3BPxrClPZHGd00tz0DWZSeDOl+4AIl4q0PQTGjH91Aafrjpf64eEAfdl1/JMJkPpjhrJW8+/DVZXBE6P6+1ZBKD4Cl7JAYBRuT9C8SyPDjH/XyotCJOhTe3CXevvhO1k4Dg2drfv0fvoHkegQKfkgocMHPkhFYZUKqm3cWmOrGvju8/fhtZUq168RXYRFlx0e5gFKqVsqampeYWkFPcRUplM5ju9vb10RU1VDRacdTvsvbYX+LMLQQktr4FACcaE4AT16Orp36eS+YsIx7r0u7ij5XtIZpOwaddvzx60tbUhlUoXcgXru63LtPJub2vTz5AKIKd4wTM3oWVPi97WIF1188xbcVL1SQF3UBL2dXRPtBfz5s0LOnYqpYYahjGd9kfqauqgeoCWT1v0ytHZibxvdiILdV2/GNihPP6jpBp+5xJs5XKgLdWGVTtWYnxxHYZEh2ix09Pdg67uLmRtG45taxFPFiqB0NXdjb1796K7u0uPpbK1/QPc9PwN+KDrfe2HkfX69UlX4LKZ8zR30EKl7PgRI0Y8TOMvu+yyXF6W33ljT0/PDMoXIna8etY1Or71oy0PDZwo5yt6FQDTxwIbFJRjGGk/XNGvbnBQFIkSyP9pzbdwbsUs/E3d32J46QhIx0F3VxfCXCDi/mBF6sWp0Na1E0+2PImXt70MFkHIGQTGtRd8W4MBL3uR8nxvCF6JMGArVqwoeEXDMMJUUjKDKWHuxXd/gbtWfR92Wdbbbz8OUkmVn6erUtIz6RMSddHTMH1YI+qH1uPE0hEoiRRrEHqyPWjrbMPm3ZvQ/Onb2LhvE5ihNI3IUo3YEdwycwFmN1yaD8ZOylqsra0NU0kJi36AwE+2jsfjOtk6yGJs3d+KRS8vRPOBt3LJ1hGWE2efx2RrnVztRS5kxvOzdE1LL9ud+tzCkJK3SJneoyfTtnFYE26+cAHGVI/RRkCQbJ1IJM6rra0tSLYeFJDgOEIsFguPI9A2L7Wv+XgN/vOdn6B591tAnB0fxxECYBy/ZqUHhJsLo8Pf3yBHGRmgYUQT/qFxPhrHN2ogkFMLJKYuHTt27Kd9f4awGPDAjm8XE4pNUsr7HccJD+xMPXkqpo2dhgM9B7Dy/TfwbutabOvchvYD7eh1e+HS3uTn+cCO9I+vSe+ew0CxiKM6Xo3ailpMrpmiwyHDKqpDp88/SUXW1JLe3t7rx48fP/iBnYE4JL8QupZl0ZG2H8Tj8emUs/qnI21HVvKOtLUkk8nrxo0b9/ahHhyUQ/ILOYqZTKbZcZyGTCYzK5lMfjMajZ4fiUT0oU8vIir+dOgz79CnHz3P2rb9q0wm88NTTjll+ZHOc1gOKRjsn8Y1TZOORVOC3dmWZdUbhqGPRXPOS49TQHqUUj1SSjoWvdlxnJXZbPa1bDbbQb4K1SM6Fg3g/wC58vyvEBd3YwAAAABJRU5ErkJggg=='

sg.theme('LightBrown11')

layout = [ # define the form layout

    [
    sg.Frame('Piston Master:',[
    [sg.Text('Frequency :'),sg.Text('Magnitude :')],
    [sg.Input(do_not_clear=True,size=(8,1)),sg.Input(do_not_clear=True,size=(8,1))],
    [sg.Button('Apply F'),sg.Button('Apply M',pad=(10,0))],

    [sg.Button('RESET Param')],
    [sg.Text('Supply       '), sg.Button('', image_data=toggle_btn_off, key='_SUPPLY_', button_color=sg.COLOR_SYSTEM_DEFAULT,border_width=0)],

    ]),

    sg.Frame('Repetitiv Control:',[
    [sg.Text('Gain :')],
    [sg.Input(do_not_clear=True,size=(8,1))],
    [sg.Button('Apply Gain')],

    [sg.Text('Enable algo'), sg.Button('', image_data=toggle_btn_off, key='_ENABLE_ALGO_', button_color=sg.COLOR_SYSTEM_DEFAULT,border_width=0)],]),

    sg.Frame('Mesures Signaux Pistons:',[
    [sg.Text('Master',font=("Helvetica", 13))],
    [sg.Text('Vmax :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_VmaxMaster-')],
    [sg.Text('Vrms :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_VrmsMaster-')],

    [sg.Text('Slave',font=("Helvetica", 13))],
    [sg.Text('Vmax :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_VmaxSlave-')],
    [sg.Text('Vrms :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_VrmsSlave-')],
    [sg.Text('ΔVrms :',font=("Helvetica", 13)),
     sg.Text(font=("Helvetica", 13),text_color='red',size=(4, 1),key='-OUTPUT_DeltaVrms-')]

    ]),

    sg.Frame('Seuil',[
    [sg.Text('ΔVrms         '), LEDIndicator('ΔVrms')],
    [sg.Text('Vmax_M        '), LEDIndicator('Vmax_M')],
    [sg.Text('Vmax_S        '), LEDIndicator('Vmax_S')],
    [sg.Text('Temperature   '), LEDIndicator('Temperature')],
    [sg.Text('H0Y ')]

    ])

    ],


    [sg.Frame('Goertzel Harmonique:',
    [[sg.Text('H0Y :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_H0-'),
     sg.Text('H1Y :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_H1-'),
     sg.Text('H2Y :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_H2-'),
     sg.Text('H3Y :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_H3-')],

    [sg.Text('H4Y :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_H4-'),
     sg.Text('H5Y :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_H5-'),
     sg.Text('H6Y :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_H6-'),
     sg.Text('H7Y :'),sg.Text(text_color='red',size=(5, 1),key='-OUTPUT_H7-')]])],


    [sg.Exit()],
    [sg.Image(r'C:/Users/anas.salamani/Desktop/air_liquide.png',pad=(90,50)),
    sg.Image(r'C:/Users/anas.salamani/Desktop/sat.png',pad=(10,0))],


    ]

window = sg.Window("CPA Control", layout,size=(1100, 650), element_justification='c',finalize=True)     # create the form


Vmax_Master_Output = window['-OUTPUT_VmaxMaster-']
Vrms_Master_Output = window['-OUTPUT_VrmsMaster-']
Vmax_Slave_Output  = window['-OUTPUT_VmaxSlave-']
Vrms_Slave_Output  = window['-OUTPUT_VrmsSlave-']
Delta_Vrms_Output  = window['-OUTPUT_DeltaVrms-']

H0_Output          = window['-OUTPUT_H0-']
H1_Output          = window['-OUTPUT_H1-']
H2_Output          = window['-OUTPUT_H2-']
H3_Output          = window['-OUTPUT_H3-']
H4_Output          = window['-OUTPUT_H4-']
H5_Output          = window['-OUTPUT_H5-']
H6_Output          = window['-OUTPUT_H6-']
H7_Output          = window['-OUTPUT_H7-']

statue_algo   = True
statue_supply = True

# ----------------------------------------------------------------------------

with serial.Serial("COM10", 250000, timeout=10) as arduino:
    time.sleep(0.01) # Time required for synchronization

    if arduino.isOpen():
        print("starting the CPA control ... ")

        while True:
            event, values = window.read(timeout=1900)

            try:
            # ---- Force/MASTER/SLAVE Acquisition from Arduino UART ----

                f_m_s_data = Acquisition()

                if max(f_m_s_data) < max_raw_value : # if no garbage value

            # -------- Splitting data, convert Raw Value to voltage ---
                    force_data, master_data, slave_data = splitting(f_m_s_data, master_data, slave_data) # Force, Master & Slave splitting
                    master_data,slave_data = Readjust_data(master_data,slave_data)                       # Remove offset & amplify signal

            # -------- Some Calculation on Signal -------
                    Vmax_master_data, Vmax_slave_data = MAX(master_data, slave_data)
                    Vrms_master_data, Vrms_slave_data = RMS(Vmax_master_data, Vmax_slave_data)
                    Delta_Vrms = abs(Vrms_master_data - Vrms_slave_data)

            # -------- FFT ------------------------------
                    FFT_master, FFT_slave = FFT(master_data,slave_data)

            # -------- GOERTZEL -------------------------
                    results = goertzel(force_data, Fech, TAB_FREQ)

                    nbr_data +=1
                    x_harm.append(nbr_data)
                    for harm in range(0,len(harmonics)):
                        harmonics[harm].append(results[harm])

            # -------- Updating Element of GUI ----------
                    Updating_output()

                    if nbr_data > 100 :
                        x_harm.pop(0)
                        for harm in range(0,len(harmonics)):
                            harmonics[harm].pop(0)
zeros

            # -------- Interaction with GUI -------------
                    if event :
                        statue_supply, statue_algo, magnitude, frequency = event_handler(event,
                                                                                         values,
                                                                                         magnitude,
                                                                                         frequency,
                                                                                         statue_supply,
                                                                                         statue_algo)

            except IndexError :
                f_m_s_data.clear()




