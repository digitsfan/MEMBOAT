#! /usr/bin/python
# connect.py: connect to MySQL server

import mysql.connector as mysql
conn_params = {
    'database': 'cookbook',
    'host': "localhost",
    'user': "root",
    'password': "xxxxx"}

try:
    conn = mysql.connect(**conn_params)
    print("Connected")
except mysql.Error as e:
    print("Cannot connect to server")
    print("Error code: %s" % e.errno)
    print("Error message: %s" % e.msg)
    print("Error SQLSTATE: %s" % e.sqlstate)
else:
    conn.close()
    print("Disconnected")
