import tkinter as tk
from networktables import NetworkTables

# ================= CONFIG =================

NT_TABLE_PATH = "AdvantageKit/RealOutputs/Display/"
LINE1_KEY = "Line1"
LINE2_KEY = "Line2"

SERVER_IP = "127.0.0.1"  # ou roborio-XXXX-frc.local
UPDATE_PERIOD_MS = 50

# ================= NETWORKTABLES =================

def nt_init():
    def connection_listener(connected, info):
        print("[NT]", "Connected" if connected else "Disconnected")

    NetworkTables.addConnectionListener(
        connection_listener, immediateNotify=True
    )

    NetworkTables.initialize(server=SERVER_IP)
    print("[NT] Initializing...")

    return NetworkTables.getTable(NT_TABLE_PATH)

# ================= LCD FORMAT =================

def lcd_format(text):
    if text is None:
        text = ""
    text = str(text)
    return text[:16].ljust(16)

# ================= GUI =================

class LCD16x2(tk.Tk):
    def __init__(self, table):
        super().__init__()
        self.table = table

        self.title("FRC LCD 16x2")
        self.configure(bg="#0b1c14")

        # === FULLSCREEN ===
        self.attributes("-fullscreen", True)
        self.bind("<Escape>", lambda e: self.destroy())  # ESC fecha

        frame = tk.Frame(
            self,
            bg="#0b1c14",
            padx=40,
            pady=40,
        )
        frame.pack(expand=True)

        self.line1 = tk.Label(
            frame,
            font=("Courier New", 64, "bold"),
            width=16,
            bg="#0f1a14",
            fg="#7CFF8A",
            bd=6,
            relief="sunken",
        )
        self.line1.pack(pady=(0, 20))

        self.line2 = tk.Label(
            frame,
            font=("Courier New", 64, "bold"),
            width=16,
            bg="#0f1a14",
            fg="#7CFF8A",
            bd=6,
            relief="sunken",
        )
        self.line2.pack()

        self.after(UPDATE_PERIOD_MS, self.update_display)

    def update_display(self):
        l1 = self.table.getString(LINE1_KEY, "")
        l2 = self.table.getString(LINE2_KEY, "")

        self.line1.config(text=lcd_format(l1))
        self.line2.config(text=lcd_format(l2))

        self.after(UPDATE_PERIOD_MS, self.update_display)


# ================= MAIN =================

def main():
    table = nt_init()
    app = LCD16x2(table)
    app.mainloop()

if __name__ == "__main__":
    main()
