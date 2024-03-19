const userInput = document.getElementById('text-input');
const checkPalindromeBtn = document.getElementById("check-btn");

const btnFunc = input =>
{
    const storeInput = input;
    if (input === '')
    {
        alert("Please input a value");
        return
    }
};

checkPalindromeBtn.addEventListener("click", () => {
    btnFunc(userInput.value);
    userInput.value = '';
});