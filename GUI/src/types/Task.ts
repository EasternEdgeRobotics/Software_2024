export type Task = {
    name: string;
    completed?: boolean;
    subTasks?: Task[];
    points?: number;
    checked?: boolean;
    };

export type SubTask = {
    name: string;
    completed: boolean;
    subTasks?: SubTask[];
    points?: number;
};