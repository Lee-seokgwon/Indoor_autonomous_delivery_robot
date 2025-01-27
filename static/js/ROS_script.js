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
            .catch(error => {
                console.error('Error:', error);
            });
            console.log("fetch 명령 보냈음!!!!!!!!!!!!!!!!!!!!!!!");
        });
    } else {
        console.error("moveRobotButton element not found.");
    }
});
