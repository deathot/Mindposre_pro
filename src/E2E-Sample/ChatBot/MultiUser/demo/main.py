import collections
import queue
import threading
import time
from multiprocessing import Process, Queue

from flask import Flask, request, jsonify
from flask import render_template  # 引入模板插件
from flask_cors import CORS

from generate import T5Model
from utils import content_generate


def run(input_queue, output_queue):
    model = T5Model()
    while True:
        if input_queue.empty():
            continue
        web_id, input_text = input_queue.get()
        print(f'[INFO] infer process get message{input_text} \n with web id{web_id}')
        model.generate_ss_mode(web_id, input_text, output_queue)


in_q = Queue(2)
out_q = Queue(1)
Process(target=run, args=(in_q, out_q)).start()
time.sleep(20)

messages_record = collections.defaultdict(list)

stream_dict = collections.defaultdict(queue.Queue)

stream_record = collections.defaultdict(str)

input_record = collections.defaultdict(str)


def worker(dic):
    """
    flask进程下的子线程，用于将消息队列中的消息取出并放入对应的消息队列中
    @param dic: 用于记录流式输出的字典，字典内部是已经默认初始化好了的队列
    """
    while True:
        if out_q.empty():
            continue
        web_id, is_finished, item, sentence = out_q.get()
        dic[web_id].put((item, is_finished, True))
        if is_finished:
            stream_record[web_id] = sentence


if __name__ == "__main__":
    threading.Thread(target=worker, args=(stream_dict,), daemon=True).start()
    app = Flask(
        __name__,
        static_folder='./dist',  # 设置静态文件夹目录
        template_folder="./dist",
        static_url_path=""
    )

    CORS(app, resources=r'/*')


    @app.route('/')
    def index():
        return render_template('index.html', name='index')


    @app.route('/chat', methods=['GET'])
    def chat():
        if request.method == 'GET':
            # Retrieve the message
            print(f'Get request: {request.args}')
            web_id = request.args.get('id')
            data = {
                "code": 200,
                "message": ''
            }
            inputs = request.args.get('message')
            print(f'Web ID: {web_id}, Message: {inputs}')
            if inputs == 'clear':
                messages_record[web_id] = []
                stream_dict[web_id].put(('聊天记录已清除', True, False))
                return jsonify(data)

            with threading.Lock():
                if web_id not in messages_record:
                    messages_record[web_id] = []
                history = messages_record[web_id]
                input_record[web_id] = inputs
            print(f'History: {history}')
            model_input = content_generate(history, inputs)
            in_q.put((web_id, model_input))

            return jsonify(data)


    @app.route('/getMsg', methods=['GET'])
    def get_msg():
        web_id = request.args.get('id')
        with threading.Lock():
            if web_id not in messages_record:
                return jsonify({'code': 404, 'message': 'No messages found for this web id'})

        save = False
        if stream_dict[web_id].empty():
            data = {
                "code": 200,
                "data": {
                    "isEnd": False,
                    "message": ""
                }
            }
        else:
            msg, end, save = stream_dict[web_id].get()
            stream_dict[web_id].task_done()
            data = {
                "code": 200,
                "data": {
                    "isEnd": end,
                    "message": msg
                }
            }

        if data['data']['isEnd'] and save:
            del stream_dict[web_id]
            messages_record[web_id].append((input_record[web_id], stream_record[web_id]))
            del stream_record[web_id]
            del input_record[web_id]

        return jsonify(data)


    @app.route('/close', methods=['GET'])
    def close():
        close_time = request.args.get('time')
        web_id = request.args.get('id')
        print(f'Close time: {close_time}, ID: {web_id}')
        return jsonify({'status': 'success'}), 200


    app.run(
        host="0.0.0.0",
        port=5000
    )
