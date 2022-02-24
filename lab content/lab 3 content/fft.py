import numpy as np
import matplotlib.pyplot as plt
from scipy import pi
from scipy.fftpack import fft

N = 288
time = np.linspace(0, 10, N)
time_data = np.array([
 00938.96,
 00917.97,
 00922.85,
 00948.24,
 00926.27,
 00910.16,
 00920.41,
 00926.76,
 00925.29,
 00940.43,
 00925.78,
 00929.69,
 00912.11,
 00925.78,
 00921.39,
 00932.13,
 00934.57,
 00927.25,
 00934.08,
 00932.62,
 00926.27,
 00922.85,
 00916.50,
 00927.73,
 00923.83,
 00933.11,
 00921.39,
 00936.04,
 00556.15,
 00814.94,
 00739.75,
-00462.89,
 00976.56,
 00986.82,
 00887.21,
 00901.37,
-00144.53,
 00864.75,
 00902.34,
 00680.18,
 00848.63,
 01723.63,
-00143.07,
 00830.57,
 00914.06,
 00994.63,
 00923.34,
 00921.39,
 00853.52,
 00888.18,
 00963.87,
 01003.91,
 00922.36,
 01017.58,
 00869.14,
 00949.71,
 01459.47,
 01122.07,
 00802.25,
 00926.27,
 00944.82,
 00968.26,
 00907.23,
-00041.50,
 01999.94,
 01485.35,
 00677.25,
 00951.66,
-02000.00,
 00624.51,
 01208.98,
 00620.12,
 01567.87,
 00876.95,
 00933.59,
-00404.30,
 00960.94,
 00926.76,
 00932.13,
 00920.41,
 01999.94,
 00925.78,
 00893.07,
 00923.83,
 01353.52,
 01327.64,
 01038.57,
 00835.45,
 00970.21,
-01784.67,
 01436.04,
 01027.34,
 00945.31,
 00930.18,
 00934.08,
 00715.82,
 00995.61,
 00966.31,
 01006.84,
 00599.61,
 00915.53,
 00944.82,
 01077.15,
 01693.36,
 00914.06,
 00896.00,
 00874.02,
 00947.27,
 00908.20,
 00994.63,
 01941.41,
 01994.63,
 00340.82,
 00247.07,
 00153.32,
 00648.44,
 00939.94,
 01858.40,
 00411.62,
 00710.94,
 00949.71,
 00933.59,
 00882.81,
 01111.82,
 00909.67,
 00658.20,
-00143.55,
 00842.29,
 00793.46,
-00694.82,
 00830.57,
 00562.50,
 00025.39,
 00809.08,
 00959.96,
-00002.93,
 00904.79,
 00875.98,
 00771.00,
 00857.42,
 00916.02,
 00864.26,
 01097.17,
 00629.39,
 00709.47,
 01124.02,
 00960.45,
 00810.06,
 00928.22,
 01019.04,
 00357.91,
 00989.26,
 00315.43,
 00812.50,
 00763.67,
 01999.94,
 01041.02,
 00689.45,
 01187.50,
 00257.81,
 00886.23,
 00888.18,
 00702.64,
 00733.89,
 00755.86,
 00993.65,
 00804.20,
-00102.54,
 00849.61,
 00734.86,
 00709.47,
 00893.07,
 00996.09,
 01081.54,
 00677.25,
 01009.28,
 00917.97,
 00701.66,
 01006.84,
 00936.04,
 01015.62,
-00176.76,
 00965.33,
 00846.19,
 00940.43,
 00664.55,
 00862.79,
 00958.50,
 00995.12,
 00928.71,
 00687.01,
 01236.33,
 00898.93,
 00948.24,
 00932.13,
 00931.64,
 00861.33,
 01114.75,
-00142.09,
-01839.36,
 00924.32,
 00325.68,
 01580.08,
 00896.00,
 00576.66,
 00877.44,
 00971.68,
 00969.73,
 00931.15,
 01010.25,
 00930.18,
 00724.12,
 01035.64,
 00905.27,
 00891.11,
 01693.85,
 01049.32,
 00963.87,
 00687.99,
 00914.55,
 00932.13,
 00983.40,
 01019.04,
 01034.18,
 00929.69,
 01024.90,
 00977.05,
 00956.05,
 00873.05,
 00819.34,
 00888.67,
 00752.93,
 00871.58,
 00955.08,
 01290.53,
 01019.04,
 00911.62,
-00455.08,
 00690.43,
 00911.62,
 01083.50,
 00836.91,
 00721.68,
 00814.45,
 01030.27,
 00960.94,
 00875.98,
 00839.36,
 00722.66,
 00805.18,
 01031.74,
 01014.65,
 00908.20,
 00941.89,
 00931.15,
 00754.39,
 00787.60,
 01146.00,
 00815.43,
 00676.27,
 00804.69,
 00977.05,
 00372.56,
 00974.61,
 00791.02,
 00856.93,
 01008.79,
 00690.43,
 00971.19,
 00905.27,
 01250.49,
 00909.18,
 00951.66,
 00907.71,
 01908.69,
 01142.58,
 00623.05,
 00812.99,
 01108.40,
 00960.94,
 00583.98,
 00839.84,
 01329.10,
 01294.92,
 00810.55,
 01010.25,
 01114.75,
-00364.75,
])

plt.plot (time [0:100], time_data [0:100])
plt.title ('Time Domain Signal')
plt.xlabel ('Time')
plt.ylabel ('Amplitude')
plt.show ()

frequency = np.linspace (0.0, 512, int (N/2))

freq_data = fft(time_data)
y = 2/N * np.abs (freq_data [0:np.int (N/2)])

plt.plot(frequency, y)
plt.title('Frequency domain Signal')
plt.xlabel('Frequency in Hz')
plt.ylabel('Amplitude')
plt.show()