# -*- coding: utf-8 -*-
"""
Created on Sun Jul 30 13:25:28 2023

@author: ahuss
"""
"""
GPS user interface.
Used to complete mission waypoints using geospatial coordinate.
DESIGN IS NOT COMPLETE
REQUIRES GOOGLE MAPS API KEY TO WORK (line 22)
"""


#things to install prior to running code googlemap and numpy libraries
import tkinter as tk
from tkinter import ttk
from tkinter import Canvas
from PIL import ImageTk, Image, ImageDraw
# import ttkbootstrap as ttk # install later
import googlemaps
import requests
import numpy as np
from datetime import datetime
import os.path
import csv

#Google authentication process
API_KEY = ''
url = "https://maps.googleapis.com/maps/api/staticmap?"

#Adjust this later on for the gps code using Serial or I2C libraries
lat = [0.0]
long = [0.0]
write_lat = [0]
write_long = [0]
GPS_array = np.zeros((1000,2))

#Creates string for Google API connection
center = ['center='+str(lat[0])+','+str(long[0])]

#Get current date for image generation
datestring = datetime.now()
date = datetime.strftime(datestring, '%Y-%m-%d')
string = 'gps-'+date

#Will make a new image of the current location on start-up or will increment the daily image
new_path = 'C:\\Users\\ahuss\\.spyder-py3\\Surface_Computer\\'+string+'.png'
new = requests.get(url+center[0]+'&zoom='+'1'+'&size=600x600&scale=1&markers=color:red%7Clabel:Start%7C'+str(lat[0])+','+str(long[0])+\
                 '&maptype=hybrid&key='+API_KEY+'&sensor=false')
#Checks to see if the path already exists and if so will make a new one iteratively then write the static map to the file
if os.path.isfile(new_path):
    counter = 1
    new_path = 'C:\\Users\\ahuss\\.spyder-py3\\Surface_Computer\\'+string+'-'+str(counter)+'.png'
    while os.path.isfile(new_path):
        counter += 1
        new_path = 'C:\\Users\\ahuss\\.spyder-py3\\Surface_Computer\\'+string+'-'+str(counter)+'.png'
    file=open(new_path,'wb')
    file.write(new.content)
    file.close()
else:
    file=open(new_path,'wb')
    file.write(new.content)
    file.close()

#function for converting lat and long pixel values
#this will only work with high zoom values (5 or greater)
def convertGPS(x, y):
    parallelMult = np.cos(lat[0] * np.pi / 180)
    degPerPixelX = 360 / np.power(2, current_zoom_value.get() + 8)
    degPerPixelY = 360 / np.power(2, current_zoom_value.get() + 8) * parallelMult
    pointLat = lat[0] - degPerPixelY * (y - 600/2)
    pointLong = long[0] + degPerPixelX * (x - 600/2)
    return ("{:.7f}".format(pointLat), "{:.7f}".format(pointLong))

#function to retrieve the current value of a slider
def get_value():
    return'{:.0f}'.format(current_zoom_value.get())
    
#function to get the pixel information for the sub_window
pixel_info = None
def get_pixel(event):
    global pixel_info
    x = event.x
    y = event.y
    data_written = False
    for i in range(len(GPS_array)):
        if GPS_array[i,0]==0 and GPS_array[i,1]==0 and y<=600:
            (GPS_array[i,0], GPS_array[i,1]) = convertGPS(x, y)
            write_lat[0] = GPS_array[i,0]
            write_long[0] = GPS_array[i,1]
            data_written = True
            print(GPS_array[i])
        if ("{:.7f}".format(GPS_array[i,0]), "{:.7f}".format(GPS_array[i,1])) == convertGPS(x, y) and data_written==True:
            break
        
    

#function for when the slider is moved to change display value
def zoom_slider_changed(event):
    zoom_value.configure(text = get_value())
    
#function for calling gps location
def get_gps():
    r = requests.get(url+center[0]+'&zoom='+str(zoom_value.cget('text'))+'&size=600x600&scale=1&markers=color:red%7Clabel:Start%7C'+str(lat[0])+','+str(long[0])+\
                     '&maptype=hybrid&key='+API_KEY+'&sensor=false')
    #Saves map image to local drive
    f = open(new_path,'wb')
    f.write(r.content)
    f.close()
    current_new_location = ImageTk.PhotoImage(Image.open(new_path))
    photo.configure(image = current_new_location)
    photo.image=current_new_location

    
def close_window(event):
    event.widget.destroy()
    
def refresh_gps_array():
    i = 0
    while GPS_array[i,0] != 0 and GPS_array[i,1] != 0:
        GPS_array[i,0] = 0
        GPS_array[i,1] = 0
        i += 1
    print('GPS points have been refreshed please draw your path again')

#Need to adjust to ensure the written values on have 6 decimal places or issues with excel can occur
def generate_csv():
    header = ['Latitude', 'Longitude']
    mission_counter = 1
    file_path = 'C:\\Users\\ahuss\\.spyder-py3\\Surface_Computer\\csv-'+date+'-mission-'+str(mission_counter)+'.csv'
    while os.path.isfile(file_path):
        mission_counter += 1
        file_path = 'C:\\Users\\ahuss\\.spyder-py3\\Surface_Computer\\csv-'+date+'-mission-'+str(mission_counter)+'.csv'
    with open(file_path,'w', newline='') as f:
        writer=csv.writer(f)
        writer.writerow(header)
        i = 0
        while GPS_array[i,0] != 0 and GPS_array[i,1] != 0:
            if GPS_array[i+1,0] == 0 and GPS_array[i+1,1] == 0:
                GPS_array[i,0] = 0
                GPS_array[i,1] = 0
                break
            writer_array = [GPS_array[i,0],GPS_array[i,1]]
            writer.writerow(writer_array)
            i += 1
        writer.writerow([0,0])
    f.close
    print('GPS point was not recorded')
    
previous_point = None

def on_click(event, location_canvas, image_on_canvas):
    global previous_point
    # Get the pixel coordinates
    x, y = event.x, event.y
    
    image = Image.open(new_path)
    # Initialize drawing
    draw = ImageDraw.Draw(image)

    # If this is the first point, draw a line from the center
    if previous_point is None:
        center_x, center_y = image.width // 2, image.height // 2
        draw.line((center_x, center_y, x, y), fill=(255, 0, 0), width=1)
    else:
        prev_x, prev_y = previous_point
        draw.line((prev_x, prev_y, x, y), fill=(255, 0, 0), width=1)
    
    # Draw a red dot (3x3 pixels)
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            if 0 <= x + dx < image.width and 0 <= y + dy < image.height:
                image.putpixel((x + dx, y + dy), (255, 0, 0))  # (255, 0, 0) is red

    # Convert the updated PIL image back to a Tkinter image
    current_location = ImageTk.PhotoImage(image)
    # Update the Tkinter display with the new image
    location_canvas.itemconfig(image_on_canvas, image=current_location)
    # Keep a reference to the new PhotoImage object
    location_canvas.image = current_location
    # Update the previous point
    previous_point = (x, y)
    # Save the image
    image.save(new_path)
    
    
def open_location_image():
    #Create a sub window when button is clicked
    sub_window = tk.Toplevel()
    sub_window.title('GPS Location')
    sub_window.geometry('600x780')
    #Return pixel information using the left mouse button
    sub_window.bind('<Button-1>', get_pixel)
    #Create image of location into sub_window
    #location_photo = ttk.Label(master=sub_window, image=photo.image)
    #location_canvas.pack(side='top', anchor='nw')
    location_canvas = Canvas(master=sub_window, width=image.width, height=image.height)
    location_canvas.pack(side='top', anchor='nw')
    
    image_on_canvas=location_canvas.create_image(0, 0, anchor=tk.NW, image=photo.image)
    location_canvas.bind("<Button-1>", lambda event: on_click(event, location_canvas, image_on_canvas))
    #Create test widget for user guidance
    draw_text = ttk.Label(master=sub_window, text='Please draw your desired path', font='Calibri 16 bold')
    draw_text.pack(side='top', anchor='n', pady=10)
    #Create secondary window
    secondary_window = ttk.Frame(master=sub_window)
    secondary_window.pack(side='top')
    #Create csv file button
    csv_button = ttk.Button(master=secondary_window, text='Generate CSV', command=generate_csv)
    csv_button.pack(side='left', anchor='n', padx= 10, pady=10)
    #Create GPS array refresh button
    gps_refresh = ttk.Button(master=secondary_window, text='Refresh GPS array', command=refresh_gps_array)
    gps_refresh.pack(side='right', anchor='n', padx=10, pady=10)
    #Recording widget
    #record_text='GPS recorded: '+str(write_lat)+', '+str(write_long)
    #record = ttk.Label(master=sub_window, textvariable=record_text, font='Calibri 16')
    #record.pack(side='top', anchor='n', pady=10)
    close_button = ttk.Label(master=sub_window, text='To close window press CTRL+Q', font='Calibri 16')
    close_button.pack(side='top', anchor='n', pady=10)
    sub_window.bind('<Control-q>', close_window)
    sub_window.mainloop()
    

def submit_gps():
    try:
        lat[0]=(float(lat_var.get()))
        long[0]=(float(long_var.get()))
        if lat[0]>90 or lat[0]<-90:
            combined_text.set("Incorrect lat values")
        elif long[0]>180 or long[0]<-180:
            combined_text.set("Incorrect long values")
        else:
            center[0] = 'center='+str(lat[0])+','+str(long[0])
            combined_text.set("Current location: "+str(lat[0])+"', "+str(long[0]))
            lat_var.set("")
            long_var.set("")
    except Exception as ex:
        print(ex)
        
    
#Window for widgets
main_window = tk.Tk()
main_window.title('User Interface')
main_window.geometry('1920x1080')


#Frame for option widgets
window_one = ttk.Frame(master=main_window, width=1020, height=980, style='TFrame')
window_one.pack(side='left',anchor='nw')

#Widgets for the slider in main window
zoom_label = ttk.Label(master=window_one, text='Google Zoom', font='Calibri 16 bold underline')
zoom_label.pack(side='top', anchor='n', padx=10)
#Widget for zoom slider
current_zoom_value = tk.DoubleVar()
zoom_slider = ttk.Scale(master=window_one, from_=1, to=20, length=325,
                        orient='horizontal', command=zoom_slider_changed, variable = current_zoom_value)
zoom_slider.set(1)
zoom_slider.pack(side='top', anchor='n', padx=10)
#Widget for displaying the slider bars value
zoom_value = ttk.Label(master=window_one, text=get_value(), font='Calibri 16')
zoom_value.pack(side='top', anchor='n', padx=20)

#Widget for dropdown label
path_label = ttk.Label(master=window_one, text='Select path profile', font='Calibri 16 bold underline')
path_label.pack(side='top', anchor='n', padx=10, pady=(10,0))
#Widget for dropdown selection menu
path_selected = tk.StringVar()
path_profile = ttk.OptionMenu(window_one, path_selected, "- Select -", 'Direct', 'Least distance traveled', 'Smooth')
path_profile.config(width=50)
path_profile.pack(side='top', anchor='n', padx=10, pady=10)

#Widget for latitude and longitude entry
entry_label = ttk.Label(master=window_one, text='GPS location', font='Calibri 16 bold underline')
entry_label.pack(side='top', anchor='n', padx=20)
#Create latitude widgets
lat_var = tk.StringVar()
lat_label = ttk.Label(master=window_one, text='Latitude: (-90<Lat<90)', font='Calibri 16')
lat_label.pack(side='top', anchor='n', pady=10)
lat_entry = ttk.Entry(master=window_one, textvariable = lat_var, font='Calibri 16')
lat_entry.pack(side='top', anchor='n', padx=10)
#create longitude widgets
long_var = tk.StringVar()
long_label = ttk.Label(master=window_one, text='Longitude: (-180<Long<180)', font='Calibri 16')
long_label.pack(side='top', anchor='n', pady=10)
long_entry = ttk.Entry(master=window_one, textvariable = long_var, font='Calibri 16')
long_entry.pack(side='top', anchor='n', padx=10)
#submit lat and long values widget
submit_button = ttk.Button(master=window_one, text='Submit', command=submit_gps)
submit_button.pack(side='top', anchor='n', padx=20, pady=10)
#current GPS location test label
combined_text = tk.StringVar()
lat_long_string = "Current location: "+str(lat[0])+", "+str(long[0])
combined_text.set(lat_long_string)
gps_location_text = tk.Label(master=window_one, textvariable=combined_text, font='Calibri 16')
gps_location_text.pack(side='top', anchor ='n', padx=10, pady=10)

#widget for display window of location
image = Image.open(new_path)
current_location = ImageTk.PhotoImage(Image.open(new_path))
window_two = ttk.Frame(master=main_window, width=900, height=1080)
window_two.pack(side='right', anchor='n', padx=10)

#Widget for labeling current location
location_label = ttk.Label(master=window_two, text='Current location', font='Calibri 16 bold underline')
location_label.pack(side='top', anchor='n')

#Widget for selecting gps location spots
select_gps = ttk.Button(master=window_two, text='Select GPS points', command=open_location_image)
select_gps.pack(side='top', anchor='n')

#widget for displaying location image
photo = ttk.Label(master=window_two, image=current_location)
photo.pack(side='top',anchor='n',pady=10)
photo.configure(image = current_location)
photo.image = current_location

#widget to refresh location image
refresh_button = ttk.Button(master=window_two, text='Refresh', command=get_gps)
refresh_button.pack(side='bottom',anchor='s', pady=10)

#input_frame = ttk.Frame(master = window)
entry_int = tk.IntVar()
#entry = ttk.Entry(master = input_frame, textvariable = entry_int)
#button = ttk.Button(master = input_frame, text = 'Whatever', command = convert)
#entry.grid(row=0,column=0)
#button.grid(row=0,column=1)
#input_frame.grid(row=1,column=0)

output_string = tk.StringVar()
#output_label = ttk.Label(master = window, text = 'Output', font = 'Calibri 24', textvariable = output_string)
#output_label.grid(row=2,column=0)


main_window.mainloop()
