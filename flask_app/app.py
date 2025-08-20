from flask import Flask, render_template
import sqlite3
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app)

db_path = "/home/azif/parol6/parol6_pipeline/pipeline_analytics.db"

@app.route('/')
def index():

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM pick_place_analytics ORDER BY id DESC LIMIT 10")
    data = cursor.fetchall()

    conn.close() 

    return render_template('index.html', data=data)

@socketio.on('connect')
def handle_connect():
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM pick_place_analytics ORDER BY id DESC LIMIT 10")
    data = cursor.fetchall()
    conn.close()


    emit('update', {'data': data})
def emit_data_periodically():
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM pick_place_analytics ORDER BY id DESC LIMIT 10")
    data = cursor.fetchall()
    conn.close()

    socketio.emit('update', {'data': data})
    socketio.sleep(5)  

if __name__ == '__main__':

    socketio.start_background_task(target=emit_data_periodically)
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)
