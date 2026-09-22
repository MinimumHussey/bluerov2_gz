# -*- coding: utf-8 -*-
"""
Created on Sat Mar 22 12:11:03 2025

@author: ahuss
"""

import tkinter as tk
from tkinter import ttk, messagebox
import csv
import datetime
import os
import paramiko

class UserProgramInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("User Program Interface")
        self.root.geometry("900x500")

        self.filename = f"{datetime.date.today().strftime('%Y_%m_%d')}.csv"

        # Top frame to hold everything
        self.top_frame = tk.Frame(root)
        self.top_frame.pack(fill="both", expand=True)

        # Left Frame for Data Entry
        self.left_frame = tk.Frame(self.top_frame, padx=10, pady=10, borderwidth=2, relief="groove")
        self.left_frame.pack(side="left", fill="y", padx=10, pady=10)

        self.create_data_entry(self.left_frame)

        # Right Frame for Data Viewer
        self.right_frame = tk.Frame(self.top_frame, padx=10, pady=10, borderwidth=2, relief="groove")
        self.right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        self.create_data_viewer(self.right_frame)

        # Bottom Frame for Send File button (below both left and right frames)
        self.bottom_frame = tk.Frame(root, padx=10, pady=10)
        self.bottom_frame.pack(fill="x", padx=10, pady=10)

        self.send_button = tk.Button(self.bottom_frame, text="Send File", command=self.send_file)
        self.send_button.pack()

    def create_data_entry(self, parent):
        degree_symbol = "\u00b0"
        self.labels = {
            "Depth (m)": "m",
            f"Heading ({degree_symbol})": degree_symbol,
            "Distance (m)": "m",
            "Surface (T/F)": ""
        }

        # Filename Entry and Read Button
        file_frame = tk.Frame(parent)
        file_frame.pack(fill="x", pady=5)

        tk.Label(file_frame, text="File Name:", width=10, anchor="w").pack(side="left")
        self.filename_entry = tk.Entry(file_frame, width=20)
        self.filename_entry.pack(side="left", padx=5)
        self.filename_entry.insert(0, self.filename)
        tk.Button(file_frame, text="Read", command=self.read_file).pack(side="left", padx=5)
        tk.Button(parent, text="Clear File", command=self.clear_file).pack(pady=5)

        self.entries = {}

        for label_text, unit in self.labels.items():
            frame = tk.Frame(parent)
            frame.pack(fill="x", pady=2)

            label = tk.Label(frame, text=label_text, width=14, anchor="w")
            label.pack(side="left")

            entry = tk.Entry(frame, width=10)
            entry.pack(side="left", padx=5)

            if unit:
                unit_label = tk.Label(frame, text=unit)
                unit_label.pack(side="left")

            self.entries[label_text] = entry

        submit_button = tk.Button(parent, text="Submit", command=self.submit_data)
        submit_button.pack(pady=10)

    def create_data_viewer(self, parent):
        self.tree = ttk.Treeview(parent)
        self.tree["columns"] = list(self.labels.keys())
        self.tree["show"] = "headings"

        for col in self.tree["columns"]:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=100, anchor="center")

        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)

        self.tree.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.load_existing_data()

    def load_existing_data(self):
        try:
            with open(self.filename, "r") as f:
                reader = csv.reader(f)
                headers = next(reader)
                for row in reader:
                    self.tree.insert("", "end", values=row)
        except FileNotFoundError:
            pass

    def submit_data(self):
    # Grab and strip all entry values
        data = [entry.get().strip() for entry in self.entries.values()]

        if data[0] == "":
            data[0] = "0"
        if float(data[0]) > 0:
            print("ERROR: POSITIVE DEPTH ENTERED")
            print("DEPTH ENTRY SET TO 0")
            data[0] = "0"

        if  data[1] == "":
            data[1] = "0"
        if float(data[1]) > 180:
            data[1] = float(data[1])
            data[1] = str(data[1]-360)
        if float(data[1]) < -180:
            data[1] = float(data[1])
            data[1] = str(data[1]+360)

        if  data[2] == "":
            data[2] = "0"

        if data[3] == "":
            data[3] = "False"
        if data[3] == "T" or data[3] == "t":
            data[3] = "True"
        if data[3] == "F" or data[3] == "f":
            data[3] = "False"

        # Handle filename
        self.filename = self.filename_entry.get().strip()
        if not self.filename.endswith(".csv"):
            self.filename += ".csv"

        file_exists = os.path.exists(self.filename)

        # Write to CSV
        with open(self.filename, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(self.labels.keys())
            writer.writerow(data)

        # Update treeview
        self.tree.insert("", "end", values=data)
        print(f"Data saved to {self.filename}: {data}")


    def read_file(self):
        new_filename = self.filename_entry.get().strip()
        if not new_filename.endswith(".csv"):
            new_filename += ".csv"
        self.filename = new_filename

        if not os.path.exists(self.filename):
            with open(self.filename, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(self.labels.keys())

        self.tree.delete(*self.tree.get_children())
        self.load_existing_data()

    def clear_file(self):
        self.filename = self.filename_entry.get().strip()
        if not self.filename.endswith(".csv"):
            self.filename += ".csv"

        with open(self.filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(self.labels.keys())

        self.tree.delete(*self.tree.get_children())
        self.load_existing_data()

    def send_file(self):
        host = "192.168.2.2"  # Replace with your Raspberry Pi IP
        port = 22
        username = "pi"
        password = "raspberry"
        remote_path = f"/home/extensions/jupyter/root/ManualControl/{self.filename}"

        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(host, port, username, password)

            sftp = ssh.open_sftp()
            sftp.put(self.filename, remote_path)
            sftp.close()
            ssh.close()

            messagebox.showinfo("Success", f"File sent to Raspberry Pi at {remote_path}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send file: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = UserProgramInterface(root)
    root.mainloop()
