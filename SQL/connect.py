#! /usr/bin/python
# connect.py: connect to MySQL server

import mysql.connector as mysql
conn_params = {
    'database': 'cookbook',
    'host': "localhost",
    'user': "root",
    'password': "ERI@N"}

try:
    conn = mysql.connect(**conn_params)
    print("Connected")
except:
    print("Cannot connect to server")
else:
    conn.close()
    print("Disconnected")
