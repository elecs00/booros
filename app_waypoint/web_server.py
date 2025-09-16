from flask import Flask, request, redirect, render_template_string, url_for
import configparser
import os

app = Flask(__name__)

INI_FILE = 'waypoints.ini'

# HTML 템플릿: UI의 전체적인 모양과 구조를 정의합니다.
# CSS를 추가하여 디자인을 개선했습니다.
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Waypoint Editor (Web)</title>
    <style>
        body { font-family: sans-serif; margin: 2em; background-color: #f4f4f9; color: #333; }
        .container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        h1, h2 { color: #444; }
        form { display: grid; grid-template-columns: 100px 1fr; gap: 10px; align-items: center; }
        label { text-align: right; padding-right: 10px; font-weight: bold; }
        input[type=text] { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }
        .buttons { grid-column: 1 / 3; display: flex; justify-content: space-between; margin-top: 10px; }
        .buttons input, .buttons a {
            padding: 10px 15px; border: none; border-radius: 4px; color: white; text-decoration: none; text-align: center; cursor: pointer; flex-grow: 1; margin: 0 5px;
        }
        .save-btn { background-color: #007bff; }
        .new-btn { background-color: #28a745; }
        hr { border: none; border-top: 1px solid #eee; margin: 20px 0; }
        ul { list-style: none; padding: 0; }
        li { background: #f9f9f9; padding: 10px; border: 1px solid #eee; margin-bottom: 5px; border-radius: 4px; display: flex; justify-content: space-between; align-items: center; }
        .actions a { margin-left: 10px; color: #dc3545; text-decoration: none; font-weight: bold; }
        .actions a.edit { color: #007bff; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Waypoint Editor</h1>
        <!-- 웨이포인트 추가 및 수정을 위한 입력 폼 -->
        <form action="/save" method="post">
            <label for="name">Name:</label>
            <input type="text" id="name" name="name" value="{{ edit_data.name }}" required>
            
            <label for="x">x:</label>
            <input type="text" id="x" name="x" value="{{ edit_data.coords.x }}">
            
            <label for="y">y:</label>
            <input type="text" id="y" name="y" value="{{ edit_data.coords.y }}">
            
            <label for="z">z:</label>
            <input type="text" id="z" name="z" value="{{ edit_data.coords.z }}">
            
            <label for="w">w:</label>
            <input type="text" id="w" name="w" value="{{ edit_data.coords.w }}">
            
            <!-- 숨겨진 필드로 이전 이름을 저장하여 이름 변경 시 사용 -->
            <input type="hidden" name="original_name" value="{{ edit_data.name }}">
            
            <div class="buttons">
                <input class="save-btn" type="submit" value="저장하기">
                <a class="new-btn" href="/">새로 추가</a>
            </div>
        </form>

        <hr>
        <h2>현재 웨이포인트 목록</h2>
        <ul>
        {% for name, coords in waypoints.items() %}
            <li>
                <span><b>{{ name }}</b>: (x: {{ coords.x }}, y: {{ coords.y }}, z: {{ coords.z }}, w: {{ coords.w }})</span>
                <span class="actions">
                    <a class="edit" href="/?edit={{ name }}">수정</a>
                    <a href="/delete/{{ name }}" onclick="return confirm('정말 \'{{ name }}\' 웨이포인트를 삭제하시겠습니까?');">삭제</a>
                </span>
            </li>
        {% endfor %}
        </ul>
    </div>
</body>
</html>
"""

def read_config():
    """INI 파일을 읽고 ConfigParser 객체를 반환합니다."""
    config = configparser.ConfigParser()
    # ini 파일이 없으면 빈 파일을 생성합니다.
    if not os.path.exists(INI_FILE):
        with open(INI_FILE, 'w', encoding='utf-8') as f:
            pass
    config.read(INI_FILE, encoding='utf-8')
    return config

# 메인 페이지 ('/') 접속 시 실행되는 함수
@app.route('/')
def index():
    config = read_config()
    # ini 파일의 내용을 딕셔너리 형태로 변환
    waypoints_dict = {s: dict(config.items(s)) for s in config.sections()}

    # 수정할 웨이포인트의 이름을 URL 파라미터에서 가져옵니다. (예: /?edit=모나리자)
    edit_name = request.args.get('edit')
    edit_data = {'name': '', 'coords': {'x': '', 'y': '', 'z': '', 'w': ''}}

    if edit_name and edit_name in waypoints_dict:
        edit_data['name'] = edit_name
        edit_data['coords'] = waypoints_dict[edit_name]

    return render_template_string(HTML_TEMPLATE, waypoints=waypoints_dict, edit_data=edit_data)

# 폼 데이터가 '/save' 경로로 전송됐을 때 실행되는 함수
@app.route('/save', methods=['POST'])
def save():
    config = read_config()

    new_name = request.form['name']
    original_name = request.form['original_name']
    
    # 이름이 변경되었고, 원래 이름이 존재했다면 기존 섹션을 삭제
    if original_name and new_name != original_name and config.has_section(original_name):
        config.remove_section(original_name)

    # 폼에서 전송된 데이터로 좌표 딕셔너리 생성
    coords = {
        'x': request.form['x'],
        'y': request.form['y'],
        'z': request.form['z'],
        'w': request.form['w']
    }

    if not new_name:
        return "오류: 이름은 비워둘 수 없습니다!", 400
    
    # configparser 객체에 데이터 업데이트 또는 추가
    config[new_name] = coords

    # 파일에 최종적으로 저장
    with open(INI_FILE, 'w', encoding='utf-8') as configfile:
        config.write(configfile)

    # 저장 후 메인 페이지로 리디렉션
    return redirect(url_for('index'))

# '/delete/이름' 경로로 접속했을 때 실행되는 함수
@app.route('/delete/<name>')
def delete(name):
    config = read_config()

    if config.has_section(name):
        config.remove_section(name)
        with open(INI_FILE, 'w', encoding='utf-8') as configfile:
            config.write(configfile)

    return redirect(url_for('index'))

# 서버 실행
if __name__ == '__main__':
    # host='0.0.0.0' : 같은 네트워크의 다른 기기에서 접속 허용
    # port=5001 : 사용할 포트 번호
    app.run(host='0.0.0.0', port=5001, debug=True)
