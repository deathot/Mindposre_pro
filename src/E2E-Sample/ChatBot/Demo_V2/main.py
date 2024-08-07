#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import os

from filelock import FileLock
from flask import Flask, request, jsonify
from flask import render_template  # 引入模板插件
from flask_cors import CORS

temp_path = os.path.join(os.getcwd(), 'temp')
input_filepath = os.path.join(temp_path, 'input.txt')
output_filepath = os.path.join(temp_path, 'output.json')
lock_filepath = os.path.join(temp_path, 'lock.txt')
lock = FileLock(lock_filepath, timeout=5)


def write_input(text):
    with lock:
        with open(input_filepath, 'w', encoding='utf-8') as f:
            f.write(text)


if __name__ == '__main__':

    app = Flask(
        __name__,
        static_folder='./dist_chatbot_standalone',  # 设置静态文件夹目录
        template_folder="./dist_chatbot_standalone",
        static_url_path=""
    )

    CORS(app, resources=r'/*')


    @app.route('/')
    def index():
        return render_template('index.html', name='index')


    @app.route("/chat", methods=["GET"])
    def getChat():
        args = request.args
        message = args.get("message")
        print('msg received')
        write_input(message)
        data = {
            "code": 200,
            "message": ''
        }
        return jsonify(data)


    @app.route("/getMsg", methods=["GET"])
    def getMsg():
        if not lock.is_locked:
            try:
                with open(output_filepath, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                os.remove(output_filepath)
                print(data)
            except FileNotFoundError:
                data = {
                    "code": 200,
                    "data": {
                        "isEnd": False,
                        "message": ""
                    }
                }
        else:
            data = {
                "code": 200,
                "data": {
                    "isEnd": False,
                    "message": ""
                }
            }

        return jsonify(data)


    app.run(
        use_reloader=False,
        host="0.0.0.0",
        port=5000
    )
