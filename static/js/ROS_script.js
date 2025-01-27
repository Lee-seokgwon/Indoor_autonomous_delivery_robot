document.addEventListener('DOMContentLoaded', function() {
    console.log("HTML문서가 완전히 로드되었습니다.");
    // moveRobotButton 클릭 이벤트 처리
    var button = document.getElementById('moveRobotButton');

    if (button) {
        button.addEventListener('click', function() {
            // 서버에 POST 요청 보내기
            fetch('/summon_robot', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
            })
            .then(response => response.text())
            .then(data => {
                document.getElementById('response').innerText = 'Robot is moving to the predefined location!';
                console.log(data); // 응답 내용 출력
            })
            .catch(error => {
                document.getElementById('response').innerText = 'Error occurred while moving the robot.';
                console.error('Error:', error);
            });
        });
    } else {
        console.error("moveRobotButton element not found.");
    }
});