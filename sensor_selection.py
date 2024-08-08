def select_sensors(num_sensors):
    print("Available sensors:")
    for i in range(num_sensors):
        print(f"Sensor {i + 1}")

    selected_indices = []
    try:
        print(f"\nSelect sensors to use (maximum {num_sensors}).\nPress 0 to stop selecting.")
        while len(selected_indices) < num_sensors:
            selected_sensor = input(f"\nEnter sensor number (1-{num_sensors}), press 'a' to select all the sensors: ")
            if selected_sensor == '0':
                break
            elif selected_sensor == 'a':
                selected_indices = list(range(num_sensors))
                break
            elif selected_sensor.isdigit() and 1 <= int(selected_sensor) <= num_sensors:
                selected_sensor = int(selected_sensor)
                if selected_sensor - 1 not in selected_indices:
                    selected_indices.append(selected_sensor - 1)
                else:
                    print("Sensor already selected.")
            else:
                print(f"Please select a number between 1 and {num_sensors} or 'a' to select all.")

        return selected_indices

    except ValueError:
        print("Invalid input. Please enter a number.")