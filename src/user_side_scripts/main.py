#!/usr/bin/env python3

import rospy
import tkinter as tk
from tkinter import simpledialog, messagebox, font
from autonomous_navigation import navigation_on_map
from task_scheduler import CCU

class WarehouseManagementGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("TC200 Warehouse Management System")
        self.root.geometry("400x300")
        self.root.configure(bg="#F0F0F0")

        self.layout = tk.Frame(root, bg="#F0F0F0")

        title_font = font.Font(family="Helvetica", size=16, weight="bold")
        title_label = tk.Label(self.layout, text="TC200 Warehouse Management System", font=title_font, bg="#F0F0F0")
        title_label.pack(pady=20)

        self.autonav_button = tk.Button(self.layout, text="Autonomous Navigation", command=self.perform_autonav, width=20, bg="#4CAF50", fg="white")
        self.autonav_button.pack(pady=10)

        self.obj_detection_button = tk.Button(self.layout, text="Object Finder", command=self.perform_obj_detection, width=20, bg="#FFA500", fg="white")
        self.obj_detection_button.pack(pady=10)

        self.exit_button = tk.Button(self.layout, text="Exit", command=self.close_app, width=20, bg="#FF5733", fg="white")
        self.exit_button.pack(pady=20)

        self.layout.pack(expand=True, fill="both")

    def perform_autonav(self):
        x = simpledialog.askfloat("Autonomous Navigation", "Enter x:")
        y = simpledialog.askfloat("Autonomous Navigation", "Enter y:")
        theta = simpledialog.askfloat("Autonomous Navigation", "Enter theta:")

        if x is not None and y is not None and theta is not None:
            auto_nav = navigation_on_map()
            auto_nav.move2pose(x, y, theta)
            messagebox.showinfo("Autonomous Navigation", f"Navigation completed. x: {x:.4f}, y: {y:.4f}, theta: {theta:.4f}")

    def perform_obj_detection(self):
        messagebox.showinfo("Going to the Shelf!", "Going to the Shelf!")
        myUseCase = CCU()
        myUseCase.run()
        base_nav = navigation_on_map()
        base_nav.move2pose(-0.69, -0.51,-1.1656773)
        messagebox.showinfo("Finished!", "Finished!")

    def close_app(self):
        # Close ROS gracefully before exiting
        rospy.signal_shutdown("Exiting TC200 Warehouse Management System")
        rospy.signal_shutdown(reason="Exit GUI")
        self.root.destroy()

def main():
    rospy.init_node('tc200_gui')

    root = tk.Tk()
    app = WarehouseManagementGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()
