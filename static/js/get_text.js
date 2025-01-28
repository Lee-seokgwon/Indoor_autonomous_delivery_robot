// /static/js/script.js

function submitForm(event) {
    event.preventDefault();  // 기본 폼 제출 동작 방지

    const name = document.getElementById("name").value;

    fetch('/submit_text', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: `name=${name}`
    })
    console.log("fetch성공했습니다!")
    .then(response => response.json())
    .then(data => {
        if (data.redirect_url) {
            // 리디렉션 URL이 포함되어 있으면 해당 페이지로 리디렉션
            window.location.href = data.redirect_url;
        } else {
            alert('좌표가 정상적으로 초기화되었습니다: ' + JSON.stringify(data));
        }
    })
    .catch(error => {
        console.error('Error:', error);
    });
}
