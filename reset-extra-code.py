def send_command(command):
    print(f"Sending: {command}")
    try:
        # If homing, perform a soft reset (Ctrl-X) first, then unlock
        if command.strip() == '$H':
            try:
                print("Soft-resetting GRBL (Ctrl-X) before homing...")
                s_machine.write(b'\x18')  # Ctrl-X soft reset
                s_machine.flush()
                time.sleep(2)
                s_machine.reset_input_buffer()
                print("Unlocking GRBL after soft reset...")
                if not send_command('$X'):
                    print("Failed to unlock after soft reset")
                    return False
                time.sleep(0.5)
            except Exception as e:
                print(f"Warning: Soft reset before homing failed: {e}")

        s_machine.write((command + '\n').encode())
        s_machine.flush()
        time.sleep(0.1)
        
        timeout = time.time() + 10
        while time.time() < timeout:
            if s_machine.in_waiting:
                response = s_machine.readline().decode().strip()
                print(f"Response: {response}")
                if response == 'ok' or 'error' in response.lower():
                    if 'error' in response.lower():
                        print(f"ERROR: {response}")
                        return False
                    return True
        print("Warning: No response received")
        return False
        
    except Exception as e:
        print(f"Error sending command: {e}")
        return False