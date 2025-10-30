import csv
import threading
import time
from datetime import datetime
from pathlib import Path
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Temperature conversion formula
def convert_voltage_to_temp(v):
    return (
        329.66 * (v - 1.25) ** 3
        + 129.04 * (v - 1.25) ** 2
        + 205.26 * (v - 1.25)
        - 4.4297
    )

def list_serial_ports():
    """Returns a list of available serial ports."""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

class FancySerialDataLoggerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Serial Data Logger")
        self.root.geometry("900x750")  # increased width for two columns

        # Use a modern theme
        style = ttk.Style()
        style.theme_use("clam")

        # Data storage and state
        self.serial_port = None
        self.time_data = []     # For saving/plotting later
        self.voltage_data = []  # For saving/plotting later
        self.temp_data = []     # For saving/plotting later
        self.live_data = []     # (timestamp, temperature) for live plot (past 15 sec)
        self.temp_history = []  # For computing average of last 100 digital readings
        self.reading = False

        # Default paths and settings
        self.default_primary_path = Path(
            "Y:/ajlab/AB/PhD_research/heat_transfer_of_plunge_freezing/"
            "temperature_measurement/thermocouple_studies/esp32_datalog/csv"
        )
        self.default_secondary_path = Path.home() / "Desktop" / "ESP32DataLogs" / "csv"
        self.base_path = self.get_base_path()
        self.default_prefix = "ESP32_TemperatureLog"
        self.use_incremental_suffix = tk.BooleanVar(value=False)
        self.save_live_data = False

        # Main container frame
        container = ttk.Frame(root, padding="10")
        container.pack(fill="both", expand=True)

        # Header with Toggle Button (positioned at top right)
        header_frame = ttk.Frame(container)
        header_frame.pack(fill="x")
        header_label = ttk.Label(header_frame, text="Advanced Serial Data Logger", 
                           font=("Helvetica", 18, "bold"))
        header_label.pack(side="left", pady=10)
        self.toggle_button = ttk.Button(header_frame, text="🟢 Switch ON", command=self.toggle_recording)
        self.toggle_button.pack(side="right", padx=10, pady=10)

        self.toggle_save_live_data_button = ttk.Button(header_frame, text="❌ Not saving live data", command=self.toggle_save_live_data)
        self.toggle_save_live_data_button.pack(side="right", padx=40, pady=40)

        # Create a content frame with two columns
        content_frame = ttk.Frame(container)
        content_frame.pack(fill="both", expand=True)
        content_frame.columnconfigure(0, weight=1)
        content_frame.columnconfigure(1, weight=1)

        # Left column: File Settings and Serial Connection
        left_frame = ttk.Frame(content_frame)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        self.file_frame = ttk.LabelFrame(left_frame, text="File Settings", padding="10")
        self.file_frame.pack(fill="x", pady=5)
        # Row 0: Save Folder label and entry
        ttk.Label(self.file_frame, text="Save Folder:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.folder_entry = ttk.Entry(self.file_frame, width=40)
        self.folder_entry.grid(row=0, column=1, padx=5, pady=5)
        self.folder_entry.insert(0, str(self.base_path))
        # Row 1: Browse button below the Save Folder entry
        ttk.Button(self.file_frame, text="Browse", command=self.browse_folder).grid(row=1, column=0, columnspan=2, padx=5, pady=5)
        # Row 2: File Prefix
        ttk.Label(self.file_frame, text="File Prefix:").grid(row=2, column=0, sticky="w", padx=5, pady=5)
        self.prefix_entry = ttk.Entry(self.file_frame, width=30)
        self.prefix_entry.grid(row=2, column=1, padx=5, pady=5)
        self.prefix_entry.insert(0, self.default_prefix)
        # Row 3: Suffix checkbox
        self.suffix_checkbox = ttk.Checkbutton(self.file_frame, 
                                               text="Use Incremental Suffix (_1, _2, etc.)",
                                               variable=self.use_incremental_suffix)
        self.suffix_checkbox.grid(row=3, column=1, sticky="w", padx=5, pady=5)

        self.serial_frame = ttk.LabelFrame(left_frame, text="Serial Connection", padding="10")
        self.serial_frame.pack(fill="x", pady=5)
        ttk.Label(self.serial_frame, text="Serial Port:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        available_ports = list_serial_ports()
        self.port_selector = ttk.Combobox(self.serial_frame, values=available_ports, state="readonly", width=18)
        self.port_selector.grid(row=0, column=1, padx=5, pady=5)
        if available_ports:
            self.port_selector.set(available_ports[0])
        ttk.Label(self.serial_frame, text="Baud Rate:").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.baud_entry = ttk.Entry(self.serial_frame, width=20)
        self.baud_entry.grid(row=1, column=1, padx=5, pady=5)
        self.baud_entry.insert(0, "115200")

        # Right column: Current Temperature and Live Plot
        right_frame = ttk.Frame(content_frame)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

        self.current_temp_frame = ttk.LabelFrame(right_frame, text="Current Temperature", padding="10")
        self.current_temp_frame.pack(fill="x", pady=5)
        # Digital display: rolling average (last 100 readings) with one decimal place.
        self.current_temp_label = ttk.Label(self.current_temp_frame, text="-- °C", font=("Helvetica", 16, "bold"))
        self.current_temp_label.pack(side="top", pady=5)

        self.live_plot_frame = ttk.LabelFrame(right_frame, text="Live Plot", padding="10")
        self.live_plot_frame.pack(fill="both", expand=True, pady=5)
        # Use a slightly smaller figure and tight_layout for full visibility
        self.live_fig = Figure(figsize=(4,2.5), dpi=100)
        self.live_ax = self.live_fig.add_subplot(111)
        self.live_ax.set_title("Live Temperature (last 15 sec)")
        self.live_ax.set_xlabel("Time (s)")
        self.live_ax.set_ylabel("Temp (°C)")
        # Fixed x-axis from -15 to 0 with custom ticks
        self.live_ax.set_xlim(-15, 0)
        self.live_ax.set_xticks([-15, -10, -5, 0])
        self.live_ax.set_xticklabels(["-15", "-10", "-5", "NOW"])
        self.live_fig.tight_layout()
        self.live_canvas = FigureCanvasTkAgg(self.live_fig, master=self.live_plot_frame)
        self.live_canvas.draw()
        self.live_canvas.get_tk_widget().pack(fill="both", expand=True)

        # Control Buttons (Plot, Save, Clear) below content
        control_frame = ttk.Frame(container)
        control_frame.pack(fill="x", padx=5, pady=10)
        self.plot_button = ttk.Button(control_frame, text="Plot Data", command=self.plot_data, state="disabled")
        self.plot_button.grid(row=0, column=0, padx=10, pady=5)
        self.save_button = ttk.Button(control_frame, text="Save to CSV", command=self.save_to_csv, state="disabled")
        self.save_button.grid(row=0, column=1, padx=10, pady=5)
        self.clear_button = ttk.Button(control_frame, text="Clear Data", command=self.clear_data, state="disabled")
        self.clear_button.grid(row=0, column=2, padx=10, pady=5)

        # Simple Reading Status Display (text only)
        self.status_label = ttk.Label(container, text="", font=("Helvetica", 12, "italic"), foreground="blue")
        self.status_label.pack(fill="x", padx=5, pady=5)

        # Activity Log Frame
        log_frame = ttk.LabelFrame(container, text="Activity Log", padding="10")
        log_frame.pack(fill="both", expand=True, padx=5, pady=5)
        self.log = ScrolledText(log_frame, height=10, width=80, font=("Consolas", 10))
        self.log.pack(fill="both", expand=True)

        # Start live plot refresh
        self.refresh_live_plot()

    def get_base_path(self):
        if self.default_primary_path.exists():
            return self.default_primary_path
        self.default_secondary_path.mkdir(parents=True, exist_ok=True)
        return self.default_secondary_path

    def browse_folder(self):
        selected_folder = filedialog.askdirectory(initialdir=str(self.base_path))
        if selected_folder:
            self.folder_entry.delete(0, "end")
            self.folder_entry.insert(0, selected_folder)

    def save_to_csv(self):
        #print(self.live_data)
        if self.save_live_data:
            self.log.insert(tk.END, "Saving live data...")
        elif not self.time_data or not self.voltage_data or not self.temp_data:
            messagebox.showerror("Error", "No data to save.")
            return
        
        folder = Path(self.folder_entry.get())
        folder.mkdir(parents=True, exist_ok=True)
        prefix = self.prefix_entry.get()
        if self.use_incremental_suffix.get():
            file_path = self.get_incremental_filename(folder, prefix, ".csv")
        else:
            timestamp = datetime.now().strftime("%Y%m%d")
            file_path = folder / f"{prefix}_{timestamp}.csv"
        
        if self.save_live_data:
            with open(file_path, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Time (s)", "Temperature (C)"])
                time_first, _ = self.live_data[0]
                for time, temp in self.live_data:
                    writer.writerow([time-time_first, temp])
            messagebox.showinfo("Success", f"Data saved to {file_path}")
            timestamp_with_HMS = datetime.now().strftime("%Y%m%d_%H:%M:%S")
            self.log.insert(tk.END, f"{timestamp_with_HMS}: Saved data to {file_path}\n")
            self.log.see(tk.END)
        else:
            with open(file_path, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Time (us)", "Voltage (V)", "Temperature (C)"])
                for t, v, temp in zip(self.time_data, self.voltage_data, self.temp_data):
                    writer.writerow([t, v, temp])
            messagebox.showinfo("Success", f"Data saved to {file_path}")
            timestamp_with_HMS = datetime.now().strftime("%Y%m%d_%H:%M:%S")
            self.log.insert(tk.END, f"{timestamp_with_HMS}: Saved data to {file_path}\n")
            self.log.see(tk.END)

    def get_incremental_filename(self, folder, prefix, extension):
        i = 1
        while True:
            file_path = folder / f"{prefix}_{i}{extension}"
            if not file_path.exists():
                return file_path
            i += 1

    def toggle_recording(self):
        if not self.reading:
            self.start_recording()
            self.toggle_button.config(text="🟥 Switch OFF")
        else:
            self.stop_recording()
            self.toggle_button.config(text="🟢 Switch ON")

    def toggle_save_live_data(self):
        if not self.save_live_data:
            self.save_live_data = True
            self.toggle_save_live_data_button.config(text="📥 Saving live data")
        else:
            self.save_live_data = False
            self.toggle_save_live_data_button.config(text="❌ Not saving live data")

    def start_recording(self):
        port = self.port_selector.get()
        baudrate = self.baud_entry.get()
        if not port or not baudrate.isdigit():
            messagebox.showerror("Error", "Please enter a valid port and baud rate.")
            return
        try:
            self.serial_port = serial.Serial(port, int(baudrate), timeout=1)
            self.log.insert(tk.END, f"Connected to {port} at {baudrate} baud.\n")
            self.log.see(tk.END)
        except serial.SerialException as e:
            messagebox.showerror("Error", f"Unable to connect to port {port}: {e}")
            return
        # Disable control buttons until recording stops
        self.plot_button.config(state="disabled")
        self.save_button.config(state="disabled")
        self.clear_button.config(state="disabled")
        # Clear previous data and reset digital display
        self.time_data.clear()
        self.voltage_data.clear()
        self.temp_data.clear()
        self.live_data.clear()
        self.temp_history.clear()
        self.current_temp_label.config(text="-- °C")
        self.reading = True
        self.reading_thread = threading.Thread(target=self.read_serial_data)
        self.reading_thread.daemon = True
        self.reading_thread.start()

    def stop_recording(self):
        self.reading = False
        # Enable control buttons if data exists
        if self.time_data:
            self.plot_button.config(state="normal")
            self.save_button.config(state="normal")
            self.clear_button.config(state="normal")
        if self.save_live_data:
            self.save_button.config(state="normal")
        self.status_label.config(text="")  # Clear reading message

    def refresh_live_plot(self):
        """Refresh the live plot with data from the past 15 seconds."""
        current_time = time.time()
        filtered_data = [(t, temp) for (t, temp) in self.live_data if current_time - t <= 15]
        times_rel = [t - current_time for t, temp in filtered_data]
        temps = [temp for t, temp in filtered_data]
        self.live_ax.cla()
        self.live_ax.plot(times_rel, temps)  # only lines, no markers
        self.live_ax.set_title("Live Temperature (last 15 sec)")
        self.live_ax.set_xlabel("Time (s)")
        self.live_ax.set_ylabel("Temp (°C)")
        self.live_ax.set_xlim(-15, 0)
        self.live_ax.set_xticks([-15, -10, -5, 0])
        self.live_ax.set_xticklabels(["-15", "-10", "-5", "NOW"])
        self.live_fig.tight_layout()
        self.live_canvas.draw()
        self.root.after(1000, self.refresh_live_plot)

    def read_serial_data(self):
        start_marker = "Start of Temperature and Time Data:"
        end_marker = "End of Data"
        sample_time_marker = "Sample Time (microseconds)"
        current_temperature_marker = "Calibrated temperature:"
        try:
            while self.reading:
                line = self.serial_port.readline().decode("utf-8").strip()
                if not line:
                    continue
                if line == start_marker:
                    self.log.insert(tk.END, "Start of data received.\n")
                    self.log.see(tk.END)
                    self.status_label.config(text="Reading data...")
                    continue
                if line == end_marker:
                    self.log.insert(tk.END, "End of data received.\n")
                    self.log.see(tk.END)
                    self.status_label.config(text="")
                    self.enable_buttons()
                    continue
                if current_temperature_marker in line:
                    try:
                        temperature_value = line.split(":")[-1]
                        temperature_float = float(temperature_value)
                        current_time = time.time()
                        self.live_data.append((current_time, temperature_float))
                        # Update digital display: rolling average of last 100 readings
                        self.temp_history.append(temperature_float)
                        if len(self.temp_history) > 100:
                            self.temp_history.pop(0)
                        avg_temp = sum(self.temp_history) / len(self.temp_history)
                        self.root.after(0, self.update_current_temp_display, avg_temp)
                    except ValueError:
                        continue
                    continue
                if sample_time_marker in line:
                    self.log.insert(tk.END, line + "\n")
                    self.log.see(tk.END)
                    continue
                if len(line.split(",")) != 2:
                    continue
                try:
                    time_str, voltage_str = line.split(",")
                    time_value = int(time_str.strip())
                    voltage_value = float(voltage_str.strip())
                    temp_value = convert_voltage_to_temp(voltage_value)
                    self.time_data.append(time_value)
                    self.voltage_data.append(voltage_value)
                    self.temp_data.append(temp_value)
                except ValueError as e:
                    self.log.insert(tk.END, f"Error parsing line '{line}': {e}\n")
                    self.log.see(tk.END)
        except Exception as e:
            self.log.insert(tk.END, f"Error reading from serial port: {e}\n")
            self.log.see(tk.END)
        finally:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.reading = False
            self.root.after(0, self.stop_recording)

    def enable_buttons(self):
        self.plot_button.config(state="normal")
        self.save_button.config(state="normal")
        self.clear_button.config(state="normal")

    def update_current_temp_display(self, avg_temp):
        self.current_temp_label.config(text=f"{avg_temp:.1f}°C")

    def clear_data(self):
        if self.time_data or self.voltage_data or self.temp_data:
            confirm = messagebox.askyesno("Confirm Clear", "Data has not been saved. Do you really want to clear the data?")
            if not confirm:
                return
        self.time_data.clear()
        self.voltage_data.clear()
        self.temp_data.clear()
        self.live_data.clear()
        self.temp_history.clear()
        self.current_temp_label.config(text="-- °C")
        self.plot_button.config(state="disabled")
        self.save_button.config(state="disabled")
        self.clear_button.config(state="disabled")
        self.log.insert(tk.END, "Data cleared from memory.\n")
        self.log.see(tk.END)

    def plot_data(self):
        from scipy.ndimage import uniform_filter1d
        import numpy as np 
        if not self.time_data or not self.temp_data:
            messagebox.showerror("Error", "No data to plot.")
            return
        time_ms = [(t - self.time_data[0]) / 1e3 for t in self.time_data]
        temp_kelvin = [t + 273.15 for t in self.temp_data]
        temp_array = np.array(temp_kelvin)
        smooth_temp = uniform_filter1d(temp_array, 50)
        plt.figure(figsize=(10, 6))
        plt.plot(time_ms, smooth_temp, label="Temperature (K)")
        plt.title("Temperature vs Time")
        plt.xlabel("Time (ms)")
        plt.ylabel("Temperature (K)")
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    root = tk.Tk()
    app = FancySerialDataLoggerApp(root)
    root.mainloop()
