import sqlite3
import matplotlib.pyplot as plt
import pandas as pd
import os

# 만약 현재 작업 디렉토리에 'data' 디렉토리 없으면 새로 생성
if not os.path.exists(os.path.join(os.getcwd(), 'data')):
    os.makedirs(os.path.join(os.getcwd(), 'data'))

# 현재 작업 디렉토리의 'data' 폴더에 'company.db' 파일을 저장
DB_PATH = os.path.join(os.getcwd(), 'data', 'company.db')


def initialize_database():
    """
    데이터베이스를 초기화하고 샘플 데이터를 삽입합니다.
    기존 테이블이 있으면 삭제하고 새로 시작합니다.
    """
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # 기존 테이블 삭제 (완전 초기화)
    cursor.execute('''DROP TABLE IF EXISTS employees''')
    cursor.execute('''DROP TABLE IF EXISTS departments''')
    cursor.execute('''DROP TABLE IF EXISTS employee_with_coordinates''')  # 새로운 테이블 삭제

    # 직원 테이블 생성
    cursor.execute(''' 
        CREATE TABLE IF NOT EXISTS employees (
            name TEXT PRIMARY KEY,
            department TEXT NOT NULL
        )
    ''')

    # 부서 테이블 생성
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS departments (
            department TEXT PRIMARY KEY,
            pos_x REAL NOT NULL,
            pos_y REAL NOT NULL,
            ori_z REAL NOT NULL,
            ori_w REAL NOT NULL
        )
    ''')

    # 직원-부서-좌표 테이블 생성
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS employee_with_coordinates (
            name TEXT PRIMARY KEY,
            department TEXT NOT NULL,
            pos_x REAL NOT NULL,
            pos_y REAL NOT NULL,
            ori_z REAL NOT NULL,
            ori_w REAL NOT NULL
        )
    ''')

    # 직원 데이터 삽입
    employees = [
        ('jaeyun', 'Developing'),
        ('seokkwon', 'Developing'),
        ('kyungbin', 'Marketing'),
        ('doyun', 'R&D'),
        ('seoyun', 'R&D')
    ]

    cursor.executemany('''
        INSERT OR IGNORE INTO employees (name, department)
        VALUES (?, ?)
    ''', employees)

    # 부서 데이터 삽입
    departments = [
        ('Developing', 1.0, 2.0, 0.0, 1.0),
        ('Marketing', 3.0, 4.0, 0.0, 1.0),
        ('R&D', 5.0, 6.0, 0.0, 1.0)
    ]

    cursor.executemany('''
        INSERT OR IGNORE INTO departments (department, pos_x, pos_y, ori_z, ori_w)
        VALUES (?, ?, ?, ?, ?)
    ''', departments)

    # 직원-부서-좌표 정보를 결합하여 employee_with_coordinates 테이블에 삽입
    cursor.execute('''
        INSERT OR IGNORE INTO employee_with_coordinates (name, department, pos_x, pos_y, ori_z, ori_w)
        SELECT e.name, e.department, d.pos_x, d.pos_y, d.ori_z, d.ori_w
        FROM employees e
        JOIN departments d ON e.department = d.department
    ''')

    conn.commit()
    conn.close()
    print("DB가 Python 코드상의 샘플 값들로 초기화되었습니다.")


def display_database():
    """
    데이터베이스에 저장된 테이블 데이터를 Matplotlib으로 출력합니다.
    """
    conn = sqlite3.connect(DB_PATH)

    # 직원 데이터 조회
    employees_df = pd.read_sql_query("SELECT * FROM employees", conn)

    # 부서 데이터 조회
    departments_df = pd.read_sql_query("SELECT * FROM departments", conn)

    # 직원-부서-좌표 데이터를 조회
    employee_with_coordinates_df = pd.read_sql_query("SELECT * FROM employee_with_coordinates", conn)

    conn.close()

    # Figure 생성
    fig, ax = plt.subplots(1, 3, figsize=(21, 6))  # 1행 3열의 서브플롯 생성

    # 직원 테이블을 첫 번째 subplot에 표시
    ax[0].axis('tight')
    ax[0].axis('off')
    ax[0].table(cellText=employees_df.values, colLabels=employees_df.columns, loc='center')

    # 부서 테이블을 두 번째 subplot에 표시
    ax[1].axis('tight')
    ax[1].axis('off')
    ax[1].table(cellText=departments_df.values, colLabels=departments_df.columns, loc='center')

    # 직원-부서-좌표 테이블을 세 번째 subplot에 표시
    ax[2].axis('tight')
    ax[2].axis('off')
    ax[2].table(cellText=employee_with_coordinates_df.values, colLabels=employee_with_coordinates_df.columns, loc='center')

    plt.suptitle("Employees, Departments, and Employee with Coordinates", fontsize=16)  # 제목 설정
    plt.show()


if __name__ == '__main__':
    initialize_database()  # DB 초기화
    display_database()     # DB 데이터를 시각화
