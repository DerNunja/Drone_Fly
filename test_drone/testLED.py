from djitellopy import Tello
t = Tello()
t.connect()

print("SDK:", t.query_sdk_version())   # sollte 3.x sein
print("Battery:", t.get_battery())
print(t.send_expansion_command("sdk?"))

print(t.send_expansion_command("led 0 255 0"))          # Grün
print(t.send_expansion_command("led 255 255 255"))      # Weiß
print(t.send_expansion_command("led_text HELLO 2"))     # Laufschrift

# LED-Matrix einschalten / Farbe setzen
t.send_expansion_command("led 255 0 0")   # Rote LED auf der Matrix einschalten