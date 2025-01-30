document.addEventListener('DOMContentLoaded', function() {
    console.log("HTML 문서가 완전히 로드되었습니다.");

    // moveRobotButton 클릭 이벤트 처리
    var button = document.getElementById('moveRobotButton');

    if (button) {
        button.addEventListener('click', function() {
            // 버튼 비활성화 및 로딩 메시지 표시
            button.disabled = true;
            button.innerText = "로봇 호출 중...";

            // 서버에 POST 요청 보내기
            fetch('/summon_robot', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
            })
            .then(response => response.json())  // 🔥 JSON 응답을 받도록 수정
            .then(data => {
                if (data.redirect) {
                    console.log("로봇 호출 성공, 페이지 이동:", data.redirect);
                    window.location.href = data.redirect;  // 🔥 서버에서 받은 URL로 이동
                } else {
                    throw new Error("서버 응답에 redirect 정보가 없습니다.");
                }
            })
            .catch(error => {
                console.error('Error:', error);
                button.innerText = "로봇 호출 실패";
                button.disabled = false;  // 🔥 실패 시 즉시 버튼 활성화
            });
        });
    } else {
        console.error("moveRobotButton element not found.");
    }
});
