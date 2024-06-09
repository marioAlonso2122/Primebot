document.addEventListener('DOMContentLoaded', function() {
    const sprintTitles = document.querySelectorAll('.sprintTitle');

    sprintTitles.forEach(sprintTitle => {
        sprintTitle.addEventListener('click', function() {
            // Toggle the display of the task list in this sprint
            const tasks = this.parentNode.querySelectorAll('.task');
            tasks.forEach(task => {
                task.style.display = task.style.display === 'block' ? 'none' : 'block';
            });
        });
    });

    const taskTitles = document.querySelectorAll('.taskTitle');

    taskTitles.forEach(title => {
        title.addEventListener('click', function() {
            // Toggle the display of the description for this task
            const taskDesc = this.nextElementSibling;
            taskDesc.style.display = taskDesc.style.display === 'block' ? 'none' : 'block';
        });
    });
});
