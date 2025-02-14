import 'package:flutter/material.dart';
import 'package:responsive_dashboard/models/all_expensess_item_model.dart';
import 'package:responsive_dashboard/views/widgets/selected_and_not_selected_all_expensess_item.dart';

class AllExpensessItem extends StatelessWidget {
  const AllExpensessItem({
    super.key,
    required this.itemModel,
    required this.isSelected,
  });
  final AllExpensessItemModel itemModel;
  final bool isSelected;
  @override
  Widget build(BuildContext context) {
    return isSelected
        ? SelectedAllExpensessItem(itemModel: itemModel)
        : NotSelectedAllExpensessItem(itemModel: itemModel);
  }
}
