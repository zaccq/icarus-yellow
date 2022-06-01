import tago

my_device = tago.Device('165bdd3a-c4fb-4417-b14e-c7b41cecc7ff')

device_info = my_device.info()


def printInfo():
    print(device_info)


def send_data(string):
    res = my_device.insert(string)
    print(res)


def get_data(fil):
    res = my_device.find(fil)
    print(f"The Recieved data is: ", res)


def send_accel_data(value, type_val):
    text = {
        'variable': type_val,
        'unit': 'meters per second squared',
        'value': value
    }
    return text


def send_gyro_data(value, type_val):
    text = {
        'variable': type_val,
        'unit': 'degrees per second',
        'value': value
    }
    return text
