document.addEventListener('DOMContentLoaded', function () {
    var loginForm = document.getElementById('loginForm');

    loginForm.addEventListener('submit', function (event) {
        event.preventDefault(); // 기본 폼 제출 방지

        var userId = document.getElementById('user_id').value;
        var password = document.getElementById('password').value;

        fetch('/login', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/x-www-form-urlencoded'
            },
            body: new URLSearchParams({
                user_id: userId,
                password: password
            })
        })
        .then(response => {
            if (response.ok) {
                // 로그인 성공 시 user_id를 sessionStorage에 저장
                sessionStorage.setItem('userId', userId);
                window.location.href = '/'; // 메인 페이지로 이동
            } else {
                return response.text(); // 에러 메시지 반환
            }
        })
        .then(errorMessage => {
            if (errorMessage) {
                document.getElementById('responseMessage').innerText = errorMessage;
            }
        })
        .catch(error => {
            console.error('Error:', error);
        });
    });
});
