from turtle import mode
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import math, time
# Подключение к беспилотнику
connection_string = 'tcp:127.0.0.1:5762'  
vehicle = connect(connection_string, wait_ready=False)
# Флаги для определения завершения взлета и полета к точке B
droid = {'mode':0}  # Создаем словарь, который будет использоваться как объект


##################################################
@vehicle.on_attribute('location.global_relative_frame')
def altitude_callback(self, attr_name, value):
    global droid
    
    if droid['mode'] == 1 : 
        # режим взлета
        print(f"Высота из.: {value.alt} м")
    if droid['mode'] == 2 : 
        # режим полета в точку - тут
        print(f"Высота изменилась: {value} метров")

##################################################



# Вспомогательная функция для вычисления расстояния между точками
def get_distance_metres(location1, location2):
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return (dlat**2 + dlong**2)**0.5 * 1.113195e5

# Функция вычисления азимута на точку интереса
def get_bearing_to_point(current_location, target_location):
    d_lat = math.radians(target_location.lat - current_location.lat)
    d_lon = math.radians(target_location.lon - current_location.lon)

    y = math.sin(d_lon) * math.cos(math.radians(target_location.lat))
    x = math.cos(math.radians(current_location.lat)) * math.sin(math.radians(target_location.lat)) - \
        math.sin(math.radians(current_location.lat)) * math.cos(math.radians(target_location.lat)) * math.cos(d_lon)

    bearing = math.degrees(math.atan2(y, x))
    bearing = (bearing + 360) % 360

    return bearing

# вернуть азимут на точку исходя из текущего положения дрона и точки интереса
def get_azimuth_at_dest(target_location):
    current_location = vehicle.location.global_frame
    azimuth = get_bearing_to_point(current_location, target_location)
    return azimuth


def get_yaw_vehicle_degrees():
    return math.degrees(vehicle.attitude.yaw) % 360


# установка нужного направления на требуемый азимут
def set_yaw_takeoff(angle_c):
    # Константы P, I, D-регулятора
    Kp = 0.34  # Коэффициент пропорциональности
    Ki = 0.023  # Коэффициент интеграции
    Kd = 0.001  # Коэффициент дифференциации
    max_yaw_rate = 50  # Максимальная угловая скорость

    target_heading_deg = angle_c  # Угол в градусах
    integral = 0
    step_res = 0
    while True:
        

        # Переводим текущий азимут в градусы
        current_yaw_deg = get_yaw_vehicle_degrees()

        # Выбираем кратчайшее направление для поворота
        if target_heading_deg > current_yaw_deg:
            if target_heading_deg - current_yaw_deg > 180:
                target_heading_deg -= 360
        else:
            if current_yaw_deg - target_heading_deg > 180:
                target_heading_deg += 360

        # Рассчитываем ошибки
        error = target_heading_deg - current_yaw_deg

        # Пропорциональная составляющая
        proportional = Kp * error

        # Интегральная составляющая
        integral += Ki * error
        
        
        integral = max(-5, min(integral, 5))  # Ограничиваем интегральную составляющую
            
        # Дифференциальная составляющая
        derivative = Kd * (error - (current_yaw_deg - target_heading_deg))

        # Рассчитываем угловую скорость
        yaw_rate = proportional + integral + derivative

        # Ограничиваем угловую скорость
        yaw_rate = max(-max_yaw_rate, min(yaw_rate, max_yaw_rate))

        # Преобразуем угловую скорость в значение для канала 4
        correction = int(1500 + yaw_rate * 10)

        # Ограничиваем значение для канала 4 в пределах [1400, 1600]
        correction = max(1450, min(correction, 1550))
    
        # Записываем значение в канал 4
        vehicle.channels.overrides['4'] = correction

        time.sleep(0.5)  # Увеличиваем время ожидания
        step_res +=1
        # Обновляем текущий угол
        current_yaw_deg = get_yaw_vehicle_degrees() 
        print(f"Т-напр.: {current_yaw_deg:.3f} градусов, "
              f"К4: {vehicle.channels.overrides['4']} , "
              f"Err deg: {error:.3f} , "
              f"Err: P={proportional:.3f}, I={integral:.3f}, D={derivative:.3f}")

        # Проверяем условие завершения
        if abs(current_yaw_deg - target_heading_deg) < 0.1:
            break

    # Выключаем канал 4
    vehicle.channels.overrides['4'] = 1500
    print(f"Дрон достиг нужного направления за {step_res} шагов")

# взлет на нужную высоту
def arm_and_takeoff(target_altitude,target_location):
    # global altitude_support_task
    print("Выполняется взлет...")
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    vehicle.mode = VehicleMode("ALT_HOLD")
    while vehicle.mode != VehicleMode("ALT_HOLD"):
        time.sleep(1)

    vehicle.channels.overrides['3'] = 2000
    vehicle.channels.overrides['4'] = 1500
    while True:
        print(f"Высота: {vehicle.location.global_relative_frame.alt:.3f}")
        vehicle.channels.overrides['3'] = 2000
        
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.99:
            vehicle.channels.overrides['3'] = 1550
        
            print("Достигнута целевая высота")
            break
        
        time.sleep(1)
       
# полет на нужную точку
def fly_to_point(target_location):
    
    print("Летим в точку B в режиме AltHold")

    vehicle.channels.overrides['2'] = 1400  # Управление тангажом (pitch)
    vehicle.channels.overrides['1'] = 1500  # Управление креном (roll)
    # в полете пересчитывать время от времени угол на точку для корректировки - может и не надо
    fly_time = 0
    while True:
        fly_time +=1
        current_location = vehicle.location.global_frame
        distance_to_target = get_distance_metres(current_location, target_location)
        print(f"Расстояние до точки B: {distance_to_target}")
        current_altitude = vehicle.location.global_relative_frame.alt
        # vehicle.channels.overrides['3'] = 1500
        vehicle.channels.overrides['2'] = 1000  # Управление тангажом (pitch)
        # vehicle.channels.overrides['1'] = 1500  # Управление креном (roll)
    # Ждем завершения движения
        if not vehicle.armed:
            print('Проблема: дрон разоружился')
            break

        if current_altitude < 10:
            print("Проблема: дрон снижаеться не долетев до точки")
        if current_altitude < 1:
            print("Проблема: дрон упал")

        # Проверка достижения точки B
        if distance_to_target < 20:
            vehicle.channels.overrides['2'] = 1750  # Управление тангажом (pitch) - погасить скорость
            time.sleep(2)
            vehicle.channels.overrides['2'] = 1500  # Управление тангажом (pitch) - зависнуть
            break

        time.sleep(1)





# Куда лететь
src_latitude = 50.450739  # широта
src_longitude = 30.461242  # долгота

target_latitude = 50.443326  # широта
target_longitude = 30.448078  # долгота
target_altitude = 100  # высота полета

target_location = LocationGlobal(target_latitude, target_longitude, target_altitude)

arm_and_takeoff(target_altitude,target_location)  # Высота взлета - 100 метров

dest_azimuth = get_azimuth_at_dest(target_location)
set_yaw_takeoff(dest_azimuth)

time.sleep(5)

fly_to_point(target_location)

set_yaw_takeoff(350)
# await altitude_support_task  # Высота взлета - 100 метров
time.sleep(30)


