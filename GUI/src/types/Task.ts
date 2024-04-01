export type Task = {
    name: string;
    completed?: boolean;
    subTasks?: Task[];
    Points?: number;
    checked?: boolean;
    };

export type SubTask = {
    name: string;
    completed: boolean;
    subTasks?: SubTask[];
    Points?: number;
};