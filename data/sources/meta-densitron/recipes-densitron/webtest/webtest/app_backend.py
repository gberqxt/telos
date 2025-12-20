#!/usr/bin/env python3
import gi
import csv
import os, time
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk, GLib

CSS = b"""
button.blue {
  background-image: none;
  background-color: #1e88e5;
  color: white;
  transition: all 0.1s;
}
button.yellow {
  background-image: none;
  background-color: #fdd835;
  color: black;
  box-shadow: 0 0 20px 10px rgba(253, 216, 53, 0.8);
  margin: -5px;  /* Negative margin expands into neighbors' space */
  padding: 5px;  /* Compensate to keep button size */
}
"""

# Files used in this application (full path)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ui_file_path = os.path.join(BASE_DIR, "app_ui.ui")
csv_file_path = os.path.join(BASE_DIR, "coordinates.csv")
        

            
def stop(self):
    self.running = False

class App:
    def __init__(self):
          

        builder = Gtk.Builder()
        builder.add_from_file(str(ui_file_path))

        self.win = builder.get_object("main_window")
        #self.lbl = builder.get_object("lbl_status")
        self.touch_area = builder.get_object("touch_area")
        
        # Load button coordinates from CSV
        self.button_coords = self.load_button_coordinates(str(csv_file_path))
        
        # Store all buttons for quick access
        self.buttons = {}
        self.load_all_buttons(builder)
        
        # Rotate text for specific buttons
        self.rotate_button_text()
        
        # Initialize button styles
        self.initialize_button_styles()


        # Connect signals declared in the .ui
        builder.connect_signals(self)
        self.win.connect("destroy", Gtk.main_quit)
        
        self.active_button = None  # Track which button is currently pressed

        # === RAW TOUCH SETUP (lower-level GDK events) ===
        mask = (
            Gdk.EventMask.TOUCH_MASK
            | Gdk.EventMask.BUTTON_PRESS_MASK
            | Gdk.EventMask.BUTTON_RELEASE_MASK
            | Gdk.EventMask.POINTER_MOTION_MASK
        )
        self.touch_area.add_events(mask)
        self.touch_area.connect("event", self.on_touch_event)

        # Track currently pressed buttons
        self.pressed_buttons = set()

        # Track active touches
        self.active_touches = {}  # slot_id -> button_id
        
        # Event log file for web interface
        self.event_log_file = '/tmp/button_events.json'
        
       
        # Load CSS globally
        provider = Gtk.CssProvider()
        provider.load_from_data(CSS)
        screen = Gdk.Screen.get_default()
        Gtk.StyleContext.add_provider_for_screen(
            screen, provider, Gtk.STYLE_PROVIDER_PRIORITY_USER
        )
      
        # === Manual fullscreen - avoid Wayland fullscreen protocol
        display = Gdk.Display.get_default()
        monitor = display.get_primary_monitor() or display.get_monitor(0)
        geom = monitor.get_geometry()
        
        self.win.set_decorated(False)
        self.win.set_default_size(geom.width, geom.height)
        self.win.move(geom.x, geom.y)       
              
        self.win.show_all()
        
        time.sleep(0.1)
        
        # auto fullscreen        
        self.win.fullscreen() 
        
        Gtk.main()

        

    def load_button_coordinates(self, filename):
        """Load button coordinates from CSV file"""
        button_coords = {}
        try:
            with open(filename, 'r', encoding='utf-8-sig') as f:  # utf-8-sig to handle BOM
                reader = csv.DictReader(f)
                for row in reader:
                    button_id = row['Buttons']
                    coords_str = row['Value (x,y)']
                    # Parse coordinates like "31, 336"
                    x_str, y_str = coords_str.split(',')
                    x = int(x_str.strip())
                    y = int(y_str.strip())
                    button_coords[(x, y)] = button_id
        except Exception as e:
            print(f"Error loading coordinates: {e}")
        return button_coords
    
    def load_all_buttons(self, builder):
        """Load all button references from builder"""
        button_ids = [
            'l1', 'l2', 'l3', 'r1', 'r2', 'r3',
            '4l', '4u', '4d', '4r', '5l', '5u', '5d', '5r',
            '6l', '6u', '6d', '6r', '7l', '7u', '7d', '7r',
            '8l', '8u', '8d', '8r', '9l', '9u', '9d', '9r',
            '10l', '10u', '10d', '10r', '11l', '11u', '11d', '11r',
            '12l', '12u', '12d', '12r', '13l', '13u', '13d', '13r',
            '14l', '14u', '14d', '14r', '15l', '15u', '15d', '15r',
            '16l', '16u', '16d', '16r', '17l', '17u', '17d', '17r',
            '18l', '18u', '18d', '18r', '19l', '19u', '19d', '19r'
        ]
        
        for btn_id in button_ids:
            button = builder.get_object(btn_id)
            if button:
                self.buttons[btn_id] = button
    
    def rotate_button_text(self):
        """Rotate text 180 degrees for specific buttons"""
        buttons_to_rotate = ['l1', 'l2', 'l3']
        # Add buttons 4 to 11 with l, u, d, r suffixes
        for i in range(4, 12):
            buttons_to_rotate.extend([f"{i}l", f"{i}u", f"{i}d", f"{i}r"])
        
        for btn_id in buttons_to_rotate:
            if btn_id in self.buttons:
                button = self.buttons[btn_id]
                label = button.get_child()
                if label and isinstance(label, Gtk.Label):
                    label.set_angle(180)  # Rotate 180 degrees
    
    def initialize_button_styles(self):
        """Initialize all buttons with blue style"""
        for button in self.buttons.values():
            ctx = button.get_style_context()
            ctx.add_class("blue")
    
    def find_nearest_button(self, x, y, tolerance=50):
        """Find the nearest button to the given coordinates"""
        min_distance = float('inf')
        nearest_button = None
        
        for (coord_x, coord_y), button_id in self.button_coords.items():
            distance = ((x - coord_x) ** 2 + (y - coord_y) ** 2) ** 0.5
            if distance < min_distance and distance <= tolerance:
                min_distance = distance
                nearest_button = button_id
        
        return nearest_button
    
    def highlight_button(self, button_id, highlight=True):
        """Highlight or unhighlight a button"""
        if button_id in self.buttons:
            button = self.buttons[button_id]
            ctx = button.get_style_context()
            
            print(f"   Button ({button_id}) {highlight}")
            
            if highlight:
                ctx.remove_class("blue")
                ctx.add_class("yellow")
                self.pressed_buttons.add(button_id)
            else:
                ctx.remove_class("yellow")
                ctx.add_class("blue")
                self.pressed_buttons.discard(button_id)
    
    # ---- Raw GDK event handler for touch_area ----            
    def on_touch_event(self, widget, event):
        et = event.type

        if et == Gdk.EventType.TOUCH_BEGIN:
            x, y = event.x, event.y
            #self.lbl.set_text(f"TOUCH_BEGIN at ({x:.0f},{y:.0f})")
            print(f"TOUCH_BEGIN at ({x:.0f},{y:.0f})")
            
            # Find and highlight the nearest button
            button_id = self.find_nearest_button(x, y)
            if button_id:
                self.highlight_button(button_id, True)
                self.write_event('press', button_id, x, y)
            return True

        elif et == Gdk.EventType.TOUCH_UPDATE:
            x, y = event.x, event.y
            #self.lbl.set_text(f"TOUCH_UPDATE at ({x:.0f},{y:.0f})")
            print(f"TOUCH_UPDATE at ({x:.0f},{y:.0f})")
            self.write_event('release', button_id, x, y)
            return True

        elif et == Gdk.EventType.TOUCH_END:
            x, y = event.x, event.y
            #self.lbl.set_text(f"TOUCH_END at ({x:.0f},{y:.0f})")
            print(f"TOUCH_END at ({x:.0f},{y:.0f})")
            
            # Find and unhighlight the nearest button
            button_id = self.find_nearest_button(x, y)
            if button_id:
                self.highlight_button(button_id, False)
                self.write_event('release', button_id, x, y)
            return True

        # Mouse fallback so you can test without a touchscreen
        elif et == Gdk.EventType.BUTTON_PRESS:
            x, y = event.x, event.y
            #self.lbl.set_text(f"CLICK at ({x:.0f},{y:.0f})")
            print(f"CLICK at ({x:.0f},{y:.0f})")
            
            button_id = self.find_nearest_button(x, y)
            if button_id:
                self.highlight_button(button_id, True)
                self.write_event('press', button_id, x, y)
            return True

        elif et == Gdk.EventType.BUTTON_RELEASE:
            x, y = event.x, event.y
            print(f"BUTTON_RELEASE at ({x:.0f},{y:.0f})")
            
            button_id = self.find_nearest_button(x, y)
            if button_id:
                self.highlight_button(button_id, False)
                self.write_event('release', button_id, x, y)
            return True

        elif et == Gdk.EventType.MOTION_NOTIFY:
            # Too chatty for a label; let it bubble
            return False

        return False  # let other handlers run

    
    def on_destroy(self, widget):
        """Clean up when window is destroyed"""
        print("Shutting down...")
        Gtk.main_quit()

    def write_event(self, event_type, button_id, x, y):
        import json
        import time
        
        event = {
            'event_type': event_type,
            'button_id': button_id,
            'system_x': x,
            'system_y': y,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S.') + f"{int(time.time() * 1000) % 1000:03d}"
        }
        
        try:
            # Read existing events
            try:
                with open(self.event_log_file, 'r') as f:
                    events = json.load(f)
            except:
                events = []
            
            # Add new event (not overwrite)
            events.append(event)
            
            # Keep only last 100 events
            if len(events) > 100:
                events = events[-100:]
            
            # Write atomically
            temp_file = self.event_log_file + '.tmp'
            with open(temp_file, 'w') as f:
                json.dump(events, f)
            os.rename(temp_file, self.event_log_file)  # Atomic replace
                
            print(f"Event written: {event_type} - {button_id}")
        except Exception as e:
            print(f"Error writing event: {e}")


    def start_test_sequence(self):
        """Test function: Simulate button presses sequentially every second"""
        button_ids = list(self.buttons.keys())
        current_index = [0]
        last_button = [None]
        
        def press_next_button():
            # Release previous button first
            if last_button[0] is not None:
                self.highlight_button(last_button[0], False)
                self.write_event('release', last_button[0], 0, 0)
                print(f"TEST: Released {last_button[0]}")
                time.sleep(0.2)
                last_button[0] = None
            
            if current_index[0] < len(button_ids):
                button_id = button_ids[current_index[0]]
                
                # Simulate press
                self.highlight_button(button_id, True)
                self.write_event('press', button_id, 0, 0)
                print(f"TEST: Pressed {button_id}")
                time.sleep(0.2)
                
                last_button[0] = button_id
                current_index[0] += 1
                return True
            else:
                print("TEST: Sequence complete")
                return False
        
        GLib.timeout_add(600, press_next_button)
        print(f"TEST: Starting sequence with {len(button_ids)} buttons")
        
    def on_key_press_event(self, widget, event):
        keyval = event.keyval
        keyname = Gdk.keyval_name(keyval)
        
        if keyname == 'F11':
            if self.win.get_property('fullscreen'):
                self.win.unfullscreen()
            else:
                self.win.fullscreen()
            return True
        elif keyname == 'Escape':
            if self.win.get_property('fullscreen'):
                self.win.unfullscreen()
                return True
            else:
                self.win.destroy()
                return True
        elif keyname == 't' or keyname == 'T':
            self.start_test_sequence()
            return True
        return False

if __name__ == "__main__":
    print("Starting Touch Application...")
    app = App()      
    # app.start_test_sequence()  
    Gtk.main()
    

