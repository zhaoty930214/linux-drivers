#include "mpu6050_lib.h"

float asin_array[]={
-90.00, -84.87, -82.75, -81.11, -79.74, -78.52, -77.42, -76.41, -75.47, -74.58, -73.74, -72.94, -72.18, -71.44, -70.73, -70.05, -69.39, -68.75, -68.13, -67.52, 
-66.93, -66.35, -65.78, -65.23, -64.69, -64.16, -63.64, -63.13, -62.62, -62.13, -61.64, -61.16, -60.69, -60.23, -59.77, -59.32, -58.87, -58.43, -57.99, -57.56, 
-57.14, -56.72, -56.30, -55.89, -55.49, -55.08, -54.69, -54.29, -53.90, -53.51, -53.13, -52.75, -52.37, -52.00, -51.63, -51.26, -50.90, -50.53, -50.17, -49.82, 
-49.46, -49.11, -48.76, -48.42, -48.07, -47.73, -47.39, -47.05, -46.72, -46.39, -46.05, -45.73, -45.40, -45.07, -44.75, -44.43, -44.11, -43.79, -43.47, -43.16, 
-42.84, -42.53, -42.22, -41.91, -41.61, -41.30, -41.00, -40.69, -40.39, -40.09, -39.79, -39.49, -39.20, -38.90, -38.61, -38.32, -38.02, -37.73, -37.45, -37.16, 
-36.87, -36.58, -36.30, -36.02, -35.73, -35.45, -35.17, -34.89, -34.61, -34.33, -34.06, -33.78, -33.50, -33.23, -32.96, -32.68, -32.41, -32.14, -31.87, -31.60, 
-31.33, -31.06, -30.80, -30.53, -30.26, -30.00, -29.74, -29.47, -29.21, -28.95, -28.69, -28.42, -28.16, -27.90, -27.65, -27.39, -27.13, -26.87, -26.62, -26.36, 
-26.10, -25.85, -25.59, -25.34, -25.09, -24.83, -24.58, -24.33, -24.08, -23.83, -23.58, -23.33, -23.08, -22.83, -22.58, -22.33, -22.09, -21.84, -21.59, -21.35, 
-21.10, -20.85, -20.61, -20.37, -20.12, -19.88, -19.63, -19.39, -19.15, -18.90, -18.66, -18.42, -18.18, -17.94, -17.70, -17.46, -17.22, -16.98, -16.74, -16.50, 
-16.26, -16.02, -15.78, -15.55, -15.31, -15.07, -14.83, -14.60, -14.36, -14.12, -13.89, -13.65, -13.41, -13.18, -12.94, -12.71, -12.47, -12.24, -12.01, -11.77, 
-11.54, -11.30, -11.07, -10.84, -10.60, -10.37, -10.14, -9.90, -9.67, -9.44, -9.21, -8.97, -8.74, -8.51, -8.28, -8.05, -7.82, -7.59, -7.35, -7.12, 
-6.89, -6.66, -6.43, -6.20, -5.97, -5.74, -5.51, -5.28, -5.05, -4.82, -4.59, -4.36, -4.13, -3.90, -3.67, -3.44, -3.21, -2.98, -2.75, -2.52, 
-2.29, -2.06, -1.83, -1.60, -1.38, -1.15, -0.92, -0.69, -0.46, -0.23, 0.00, 0.23, 0.46, 0.69, 0.92, 1.15, 1.38, 1.60, 1.83, 2.06, 
2.29, 2.52, 2.75, 2.98, 3.21, 3.44, 3.67, 3.90, 4.13, 4.36, 4.59, 4.82, 5.05, 5.28, 5.51, 5.74, 5.97, 6.20, 6.43, 6.66, 
6.89, 7.12, 7.35, 7.59, 7.82, 8.05, 8.28, 8.51, 8.74, 8.97, 9.21, 9.44, 9.67, 9.90, 10.14, 10.37, 10.60, 10.84, 11.07, 11.30, 
11.54, 11.77, 12.01, 12.24, 12.47, 12.71, 12.94, 13.18, 13.41, 13.65, 13.89, 14.12, 14.36, 14.60, 14.83, 15.07, 15.31, 15.55, 15.78, 16.02, 
16.26, 16.50, 16.74, 16.98, 17.22, 17.46, 17.70, 17.94, 18.18, 18.42, 18.66, 18.91, 19.15, 19.39, 19.63, 19.88, 20.12, 20.37, 20.61, 20.85, 
21.10, 21.35, 21.59, 21.84, 22.09, 22.33, 22.58, 22.83, 23.08, 23.33, 23.58, 23.83, 24.08, 24.33, 24.58, 24.83, 25.09, 25.34, 25.59, 25.85, 
26.10, 26.36, 26.62, 26.87, 27.13, 27.39, 27.65, 27.90, 28.16, 28.42, 28.69, 28.95, 29.21, 29.47, 29.74, 30.00, 30.26, 30.53, 30.80, 31.06, 
31.33, 31.60, 31.87, 32.14, 32.41, 32.68, 32.96, 33.23, 33.50, 33.78, 34.06, 34.33, 34.61, 34.89, 35.17, 35.45, 35.73, 36.02, 36.30, 36.58, 
36.87, 37.16, 37.45, 37.73, 38.02, 38.32, 38.61, 38.90, 39.20, 39.49, 39.79, 40.09, 40.39, 40.69, 41.00, 41.30, 41.61, 41.91, 42.22, 42.53, 
42.84, 43.16, 43.47, 43.79, 44.11, 44.43, 44.75, 45.07, 45.40, 45.73, 46.05, 46.39, 46.72, 47.05, 47.39, 47.73, 48.07, 48.42, 48.76, 49.11, 
49.46, 49.82, 50.17, 50.53, 50.90, 51.26, 51.63, 52.00, 52.37, 52.75, 53.13, 53.51, 53.90, 54.29, 54.69, 55.08, 55.49, 55.89, 56.30, 56.72, 
57.14, 57.56, 57.99, 58.43, 58.87, 59.32, 59.77, 60.23, 60.69, 61.16, 61.64, 62.13, 62.62, 63.13, 63.64, 64.16, 64.69, 65.23, 65.78, 66.35, 
66.93, 67.52, 68.13, 68.75, 69.39, 70.05, 70.73, 71.44, 72.18, 72.94, 73.74, 74.58, 75.47, 76.41, 77.42, 78.52, 79.74, 81.11, 82.75, 84.87,
90.00
};
