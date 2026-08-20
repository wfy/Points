import numpy as np
import laspy
from modules.config import ClassificationCode

class PointCloudEvaluator:
    """
    点云分类结果定量评估器
    映射规则 (ASPRS & 国家电网规范):
      Ground: 2
      Cable: 13 (Wire-Guard), 14 (Wire-Conductor)
      Tower: 15 (Transmission Tower)
      Vegetation/Other: 0, 1, 3, 4, 5, 18 等
    """
    CLASS_NAMES = ['Ground', 'Cable', 'Tower', 'Vegetation']

    @classmethod
    def map_classification_to_label(cls, class_array: np.ndarray) -> np.ndarray:
        """
        将 LAS 标准 Classification 代码映射为通用类别索引 (0: Ground, 1: Cable, 2: Tower, 3: Vegetation)
        """
        labels = np.full(len(class_array), 3, dtype=int) # 默认 3: Vegetation
        labels[np.isin(class_array, [int(ClassificationCode.GROUND)])] = 0
        labels[np.isin(class_array, [int(ClassificationCode.WIRE_GUARD), int(ClassificationCode.WIRE_CONDUCTOR)])] = 1
        labels[np.isin(class_array, [int(ClassificationCode.TRANSMISSION_TOWER)])] = 2
        return labels

    @classmethod
    def evaluate_classifications(cls, y_true: np.ndarray, y_pred: np.ndarray):
        """
        根据标注类别 y_true 与预测类别 y_pred 计算混淆矩阵与性能指标
        """
        labels_true = cls.map_classification_to_label(y_true)
        labels_pred = cls.map_classification_to_label(y_pred)
        
        class_names = cls.CLASS_NAMES
        num_classes = len(class_names)
        
        cm = np.zeros((num_classes, num_classes), dtype=int)
        for t, p in zip(labels_true, labels_pred):
            cm[t, p] += 1
            
        metrics = {}
        total_correct = np.trace(cm)
        total_samples = len(y_true)
        overall_acc = total_correct / max(total_samples, 1)
        
        for i, name in enumerate(class_names):
            tp = cm[i, i]
            fp = np.sum(cm[:, i]) - tp
            fn = np.sum(cm[i, :]) - tp
            
            precision = tp / max(tp + fp, 1)
            recall = tp / max(tp + fn, 1)
            f1 = 2 * precision * recall / max(precision + recall, 1e-6)
            
            metrics[name] = {
                'TP': int(tp),
                'FP': int(fp),
                'FN': int(fn),
                'Precision': float(precision),
                'Recall': float(recall),
                'F1-Score': float(f1)
            }
            
        return {
            'Overall_Accuracy': float(overall_acc),
            'Confusion_Matrix': cm.tolist(),
            'Class_Metrics': metrics
        }

    @classmethod
    def evaluate_las_files(cls, ground_truth_las_path: str, predicted_las_path: str):
        """
        直接读取标注 LAS 与预测 LAS 并计算评估指标
        """
        las_gt = laspy.read(ground_truth_las_path)
        las_pred = laspy.read(predicted_las_path)
        
        if len(las_gt.points) != len(las_pred.points):
            raise ValueError(f"真值点云数 ({len(las_gt.points)}) 与预测点云数 ({len(las_pred.points)}) 不匹配！")
            
        return cls.evaluate_classifications(las_gt.classification, las_pred.classification)

def print_evaluation_report(report: dict, title: str = "点云电力线分类评估报告"):
    """
    格式化输出评估报表
    """
    print(f"\n=======================================================")
    print(f" [REPORT] {title}")
    print(f"=======================================================")
    print(f" 整体准确率 (Overall Accuracy): {report['Overall_Accuracy'] * 100:.2f}%\n")
    
    headers = ["类别 (Class)", "Precision", "Recall", "F1-Score", "TP (真阳)", "FP (假阳)", "FN (假阴)"]
    print(f"{headers[0]:<15} | {headers[1]:<10} | {headers[2]:<10} | {headers[3]:<10} | {headers[4]:<10} | {headers[5]:<10} | {headers[6]:<10}")
    print("-" * 85)
    
    for cls_name, m in report['Class_Metrics'].items():
        print(f"{cls_name:<15} | {m['Precision']*100:>8.2f}% | {m['Recall']*100:>8.2f}% | {m['F1-Score']*100:>8.2f}% | {m['TP']:<10} | {m['FP']:<10} | {m['FN']:<10}")
    print(f"=======================================================\n")
