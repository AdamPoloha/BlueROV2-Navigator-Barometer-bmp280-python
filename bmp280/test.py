#!/usr/bin/python3

def main():
    from bmp280 import BMP280

    device = "bmp280"
    bmp = BMP280()

    while True:
        #compensation = bmp.get_compensation()

        def data_getter():
            data = bmp.get_data()
            #print(f'{data.pressure:.6f} {data.temperature:.6f} {data.pressure_raw} {data.temperature_raw}')
            print(data.pressure, data.temperature, data.pressure_raw, data.temperature_raw)
            #bmp.close()
        
        data_getter()

if __name__ == '__main__':
    main()
