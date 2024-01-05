import serial
import csv
import time

# Ustawienia portu szeregowego (zmień odpowiednio)
ser = serial.Serial('COM4', 9600, timeout=1)

# Nazwa pliku CSV do zapisu
filename = 'pomiary_czujnika.csv'

# Otwarcie pliku do zapisu
with open(filename, 'w', newline='') as csvfile:
    # Utworzenie obiektu do zapisu CSV
    csvwriter = csv.writer(csvfile)

    # Zapis nagłówka pliku CSV
    csvwriter.writerow(['Time', 'Temperature'])

    try:
        while True:
            # Odczytanie danych z portu szeregowego
            line = ser.readline().decode('utf-8').strip()

            # Pomiń puste linie
            if not line:
                continue

            # Pobranie bieżącego czasu
            current_time = time.strftime('%H:%M:%S')

            # Podzielenie danych na temperaturę i wilgotność
            temperature = map(float, line.split(','))

            # Zapis pomiaru do pliku CSV
            csvwriter.writerow([current_time, temperature])

            # Pauza (można dostosować do potrzeb)
            time.sleep(1)

    except KeyboardInterrupt:
        print("Zakończono pomiary.")

    finally:
        # Zamknięcie portu szeregowego
        ser.close()