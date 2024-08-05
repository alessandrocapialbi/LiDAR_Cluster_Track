def select_sensors(num_sensors):
    print("Available sensors:")
    for i in range(num_sensors):
        print(f"Sensor {i + 1}")

    selected_indices = []
    try:
        print(f"Select sensors to use (maximum {num_sensors}). Press 0 to stop selecting.\n\n")
        while len(selected_indices) < num_sensors:
            selected_sensor = int(input(f"Enter sensor number (1-{num_sensors}) or 0 to stop: "))
            if selected_sensor == 0:
                break
            elif 1 <= selected_sensor <= num_sensors:
                if selected_sensor - 1 not in selected_indices:
                    selected_indices.append(selected_sensor - 1)
                else:
                    print("Sensor already selected.")
            else:
                print(f"Please select a number between 1 and {num_sensors}.")

        return selected_indices

    except ValueError:
        print("Invalid input. Please enter a number.")
