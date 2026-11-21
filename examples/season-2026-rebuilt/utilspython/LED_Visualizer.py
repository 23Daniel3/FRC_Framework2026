import tkinter as tk
from networktables import NetworkTables

# ================= CONFIG =================

SERVER_IP = "127.0.0.1"

ROOT_TABLE = "AdvantageKit"
LED_TABLE = "Led"

UPDATE_MS = 40

LED_SIZE = 16
LED_GAP = 8
MARGIN = 20

MIN_LEDS_PER_ROW = 10

BG_COLOR = "#0d0f12"
LED_OUTLINE = "#1f1f1f"

# ================= NT CACHE =================

led_length = 0
nt_connected = False

red_array = []
green_array = []
blue_array = []

# ================= NETWORKTABLES =================

def nt_init():
    global led_length, red_array, green_array, blue_array, nt_connected

    def on_connect(connected, info):
        global nt_connected
        nt_connected = connected

    NetworkTables.addConnectionListener(on_connect, immediateNotify=True)
    NetworkTables.initialize(server=SERVER_IP)

    ak = NetworkTables.getTable(ROOT_TABLE)
    led = ak.getSubTable(LED_TABLE)

    def length_listener(table, key, value, isNew):
        global led_length
        if key == "Length":
            led_length = int(value)

    led.addEntryListener(length_listener, immediateNotify=True)

    def array_listener(table, key, value, isNew):
        global red_array, green_array, blue_array
        if key == "Red":
            red_array = [int(v) for v in value]
        elif key == "Green":
            green_array = [int(v) for v in value]
        elif key == "Blue":
            blue_array = [int(v) for v in value]

    led.addEntryListener(array_listener, immediateNotify=True)

# ================= GUI =================

class LedStripSim(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("FRC LED Visualizer")
        self.configure(bg=BG_COLOR)
        self.minsize(600, 300)

        self.fullscreen = False

        # ---------- Header ----------
        self.header = tk.Frame(self, bg=BG_COLOR)
        self.header.pack(fill="x", padx=16, pady=(12, 6))

        self.status_label = tk.Label(
            self.header,
            text="● NT: DISCONNECTED",
            fg="#ff5555",
            bg=BG_COLOR,
            font=("Segoe UI", 10, "bold")
        )
        self.status_label.pack(side="left")

        self.length_label = tk.Label(
            self.header,
            text="LEDs: 0",
            fg="#cccccc",
            bg=BG_COLOR,
            font=("Segoe UI", 10)
        )
        self.length_label.pack(side="right")

        # ---------- Canvas ----------
        self.canvas = tk.Canvas(self, bg=BG_COLOR, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        self.led_items = []
        self.led_positions = []

        self.current_length = 0
        self.current_cols = 0

        # bindings
        self.bind("<Configure>", self.on_resize)
        self.bind("<F11>", self.toggle_fullscreen)
        self.bind("<Escape>", self.exit_fullscreen)

        self.after(UPDATE_MS, self.update_strip)

    # ---------- Layout ----------
    def compute_columns(self):
        width = self.canvas.winfo_width()
        if width <= 0:
            return MIN_LEDS_PER_ROW

        usable = width - MARGIN * 2
        cols = usable // (LED_SIZE + LED_GAP)
        return max(MIN_LEDS_PER_ROW, int(cols))

    def rebuild_strip(self):
        self.canvas.delete("all")
        self.led_items.clear()
        self.led_positions.clear()

        cols = self.compute_columns()
        rows = (self.current_length + cols - 1) // cols

        for i in range(self.current_length):
            row = i // cols
            col = i % cols

            # zig-zag físico
            if row % 2 == 1:
                col = cols - 1 - col

            x = MARGIN + col * (LED_SIZE + LED_GAP)
            y = MARGIN + row * (LED_SIZE + LED_GAP)

            # glow
            self.canvas.create_oval(
                x - 3, y - 3,
                x + LED_SIZE + 3, y + LED_SIZE + 3,
                fill="#070707",
                outline=""
            )

            led = self.canvas.create_oval(
                x, y,
                x + LED_SIZE, y + LED_SIZE,
                fill="#000000",
                outline=LED_OUTLINE
            )

            self.led_items.append(led)
            self.led_positions.append((x, y))

        self.current_cols = cols

    # ---------- Events ----------
    def on_resize(self, event):
        cols = self.compute_columns()
        if cols != self.current_cols and self.current_length > 0:
            self.rebuild_strip()

    def toggle_fullscreen(self, event=None):
        self.fullscreen = not self.fullscreen
        self.attributes("-fullscreen", self.fullscreen)

    def exit_fullscreen(self, event=None):
        self.fullscreen = False
        self.attributes("-fullscreen", False)

    # ---------- Update ----------
    def update_strip(self):
        global led_length, red_array, green_array, blue_array, nt_connected

        self.status_label.config(
            text="● NT: CONNECTED" if nt_connected else "● NT: DISCONNECTED",
            fg="#4cff4c" if nt_connected else "#ff5555"
        )

        if led_length > 0 and led_length != self.current_length:
            self.current_length = led_length
            self.length_label.config(text=f"LEDs: {led_length}")
            self.rebuild_strip()

        for i in range(self.current_length):
            r = red_array[i] if i < len(red_array) else 0
            g = green_array[i] if i < len(green_array) else 0
            b = blue_array[i] if i < len(blue_array) else 0

            self.canvas.itemconfig(
                self.led_items[i],
                fill=f"#{r:02x}{g:02x}{b:02x}"
            )

        self.after(UPDATE_MS, self.update_strip)

# ================= MAIN =================

def main():
    nt_init()
    app = LedStripSim()
    app.mainloop()

if __name__ == "__main__":
    main()
