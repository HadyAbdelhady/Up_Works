import 'package:flutter/material.dart';
import 'package:flutter_svg/flutter_svg.dart';
import 'package:responsive_dashboard/models/user_info_model.dart';
import 'package:responsive_dashboard/utils/app_styles.dart';

class UserInfoListTile extends StatelessWidget {
  const UserInfoListTile({
    super.key,
    required this.userInfoModel,
  });

  final UserInfoModel userInfoModel;
  @override
  Widget build(BuildContext context) {
    return IntrinsicWidth(
      child: Card(
        elevation: 0,
        color: const Color(0xFFFAFAFA),
        child: Center(
          child: ListTile(
            title: Text(
              userInfoModel.title,
              style: AppStyles.styleBold16(context),
            ),
            subtitle: Text(
              userInfoModel.subTitle,
              style: AppStyles.styleRegular12(context),
            ),
            leading: SvgPicture.asset(
              userInfoModel.image,
            ),
          ),
        ),
      ),
    );
  }
}
