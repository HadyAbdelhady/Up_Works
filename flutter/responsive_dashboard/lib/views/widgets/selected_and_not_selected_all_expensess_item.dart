import 'package:flutter/material.dart';
import 'package:responsive_dashboard/models/all_expensess_item_model.dart';
import 'package:responsive_dashboard/utils/app_styles.dart';
import 'package:responsive_dashboard/views/widgets/all_expensess_item_header.dart';

class NotSelectedAllExpensessItem extends StatelessWidget {
  const NotSelectedAllExpensessItem({
    super.key,
    required this.itemModel,
  });

  final AllExpensessItemModel itemModel;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
      decoration: ShapeDecoration(
        color: Colors.white,
        shape: RoundedRectangleBorder(
          side: const BorderSide(width: 1, color: Color(0xFFF1F1F1)),
          borderRadius: BorderRadius.circular(12),
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          AllExpensessItemHeader(
            image: itemModel.image,
          ),
          const SizedBox(
            height: 34,
          ),
          Text(
            itemModel.title,
            style: AppStyles.styleSemiBold16(context),
          ),
          const SizedBox(
            height: 8,
          ),
          Text(
            itemModel.data,
            style: AppStyles.styleRegular14(context),
          ),
          const SizedBox(
            height: 16,
          ),
          Text(
            itemModel.price.toString(),
            style: AppStyles.styleSemiBold24(context),
          )
        ],
      ),
    );
  }
}

class SelectedAllExpensessItem extends StatelessWidget {
  const SelectedAllExpensessItem({
    super.key,
    required this.itemModel,
  });

  final AllExpensessItemModel itemModel;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
      decoration: ShapeDecoration(
        color: const Color(0xFF4DB7F2),
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(12),
        ),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          AllExpensessItemHeader(
            imageBackground: Colors.white.withOpacity(0.10000000149011612),
            imageColor: Colors.white,
            image: itemModel.image,
          ),
          const SizedBox(
            height: 34,
          ),
          Text(
            itemModel.title,
            style: AppStyles.styleSemiBold16(context).copyWith(
              color: Colors.white,
            ),
          ),
          const SizedBox(
            height: 8,
          ),
          Text(
            itemModel.data,
            style: AppStyles.styleRegular14(context).copyWith(
              color: const Color(0xFFFAFAFA),
            ),
          ),
          const SizedBox(
            height: 16,
          ),
          Text(
            itemModel.price.toString(),
            style: AppStyles.styleSemiBold24(context).copyWith(
              color: Colors.white,
            ),
          )
        ],
      ),
    );
  }
}
