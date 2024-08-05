def select_sensor(point_clouds):
    num_sensors = len(point_clouds)
    print("Available sensors:")
    for i in range(num_sensors):
        print(f"Sensor {i + 1}")

    while True:
        try:
            selected_sensor = int(input(f"Select a sensor (1-{num_sensors}): ")) - 1
            if 0 <= selected_sensor < num_sensors:
                return point_clouds[selected_sensor]
            else:
                print(f"Please select a number between 1 and {num_sensors}.")
        except ValueError:
            print("Invalid input. Please enter a number.")
