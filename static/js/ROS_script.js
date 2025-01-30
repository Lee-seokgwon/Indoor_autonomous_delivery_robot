document.addEventListener('DOMContentLoaded', function() {
    console.log("HTML문서가 완전히 로드되었습니다.");

    // moveRobotButton 클릭 이벤트 처리
    var button = document.getElementById('moveRobotButton');

    if (button) {
        button.addEventListener('click', function() {
            // 버튼을 클릭한 후 사용자에게 로딩 표시를 제공하기 위해 버튼 비활성화
            button.disabled = true;
            button.innerText = "로봇 호출 중...";

            // 로딩 스피너를 표시하려면 여기에서 추가 가능
            // 예: showLoadingSpinner();

            // 서버에 POST 요청 보내기
            fetch('/summon_robot', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
            })
            .then(response => {
                // 응답이 성공적이라면
                if (response.ok) {
                    console.log('로봇 호출 성공');
                    // 성공적으로 호출된 후 버튼 텍스트 변경 및 버튼 재활성화
                    button.innerText = "로봇 호출 완료";
                } else {
                    // 응답이 실패한 경우 처리
                    console.error('서버 응답 오류:', response.statusText);
                    button.innerText = "로봇 호출 실패";
                }
            })
            .catch(error => {
                // 네트워크 오류 처리
                console.error('Error:', error);
                button.innerText = "로봇 호출 실패";
            })
            .finally(() => {
                // 요청이 끝나면 버튼을 다시 활성화
                setTimeout(() => {
                    button.disabled = false;
                }, 3000);  // 3초 후 버튼 활성화
            });
        });
    } else {
        console.error("moveRobotButton element not found.");
    }
});
